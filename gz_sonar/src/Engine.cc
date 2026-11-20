/*
 * Copyright (C) 2026 Honu Robotics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#include "gz/maritime/sonar/Engine.hh"

#include <algorithm>
#include <cmath>
#include <thread>

#include "gz/maritime/sonar/Random.hh"

namespace gz::maritime::sonar
{
namespace
{
/// \brief sinc^2(x) = 0.5 at this x.
constexpr double kHalfPowerX = 0.442946470689452;

/// \brief First sidelobe of sinc^2, dB.
constexpr double kSincSidelobeDb = -13.2614;

/// \brief Weights below this are dropped: -50 dB.
constexpr double kPatternFloor = 1e-5;

/// \brief Never divide by less than this cosine: incidence just short of
/// grazing.
constexpr double kMinCos = 1e-3;

/// \brief A footprint spreads over at most this many samples: more smears
/// a target's edge, where rays graze it, far in range.
constexpr double kMaxFootprintSamples = 8.0;

double Sinc2(double _x)
{
  if (std::abs(_x) < 1e-9)
    return 1.0;
  const double s = std::sin(M_PI * _x) / (M_PI * _x);
  return s * s;
}

/// \brief Run _fn(i) for i in [0, _n) on up to _threads threads, each
/// taking a contiguous block. _fn must only touch what belongs to i.
template<typename Fn>
void ParallelFor(std::size_t _n, std::size_t _threads, Fn _fn)
{
  const std::size_t t = std::min(_threads, _n);
  if (t <= 1)
  {
    for (std::size_t i = 0; i < _n; ++i)
      _fn(i);
    return;
  }
  std::vector<std::thread> pool;
  pool.reserve(t);
  for (std::size_t w = 0; w < t; ++w)
  {
    pool.emplace_back([&, w]() {
      for (std::size_t i = w * _n / t; i < (w + 1) * _n / t; ++i)
        _fn(i);
    });
  }
  for (auto &th : pool)
    th.join();
}

/// \brief Stream numbers for the random draws of a ping.
constexpr std::uint64_t kSpeckleStream = 1;
}  // namespace

double HorizontalPattern(const SonarConfig &_config, double _offset)
{
  const double theta0 = 0.5 * _config.beamWidth / kHalfPowerX;
  const double x = _offset / theta0;
  const double w = Sinc2(x);
  if (std::abs(x) <= 1.0)
    return w;
  return w * std::pow(10.0,
    (_config.sidelobeLevelDb - kSincSidelobeDb) / 10.0);
}

double VerticalPattern(const SonarConfig &_config, double _elevation)
{
  const double u = 2.0 * _elevation / _config.verticalFov;
  return std::exp(-M_LN2 * u * u);
}

Engine::Engine(const SonarConfig &_config, const Water &_water)
  : config(_config), water(_water),
    alpha(AbsorptionDbPerMeter(_config.frequency, _water))
{
}

const SonarConfig &Engine::Config() const
{
  return this->config;
}

const Water &Engine::WaterProperties() const
{
  return this->water;
}

double Engine::Absorption() const
{
  return this->alpha;
}

BeamIntensity Engine::Ping(const std::vector<RayHit> &_hits,
  std::uint64_t _ping) const
{
  const SonarConfig &c = this->config;
  BeamIntensity out;
  out.beams = c.beams;
  out.samples = c.samples;
  out.data.assign(c.beams * c.samples, 0.0);
  out.labels.assign(c.beams * c.samples, 0);
  for (std::size_t k = 0; k < c.samples; ++k)
    out.ranges.push_back(c.SampleRange(k));
  for (std::size_t j = 0; j < c.beams; ++j)
    out.azimuths.push_back(c.BeamAzimuth(j));

  // The strongest contribution each cell has seen, for its label.
  std::vector<double> best(c.beams * c.samples, 0.0);

  const double assumed = c.AssumedSoundSpeed(this->water.soundSpeed);
  const double rangeScale = assumed / this->water.soundSpeed;
  const double dr = c.SampleSpacing();
  const double pulseRange = 0.5 * assumed * c.pulseLength;
  const double beamSpacing = c.horizontalFov / static_cast<double>(c.beams);

  // How far, in beams, the horizontal pattern stays above the floor.
  double reach = c.beamWidth;
  while (reach < c.horizontalFov + 4.0 * c.beamWidth)
  {
    // Sidelobe peaks fall as 1 / x^2; stop where even a peak is negligible.
    const double theta0 = 0.5 * c.beamWidth / kHalfPowerX;
    const double x = reach / theta0;
    const double peak = 1.0 / (M_PI * M_PI * x * x) *
      std::pow(10.0, (c.sidelobeLevelDb - kSincSidelobeDb) / 10.0);
    if (peak < kPatternFloor)
      break;
    reach += c.beamWidth;
  }
  const auto beamReach = static_cast<long>(std::ceil(reach / beamSpacing));

  // Rays that share an azimuth share their beam weights, so the energy is
  // first summed into fine azimuth columns (the ray fan's spacing) and each
  // beam then gathers from the columns its pattern reaches. Same sums, far
  // fewer of them.
  const double colStep = std::min(beamSpacing, c.beamWidth) /
    static_cast<double>(std::max<std::size_t>(c.raysPerBeam, 1));
  const double azHalf = 0.5 * c.horizontalFov + 2.0 * c.beamWidth;
  const auto cols = static_cast<std::size_t>(std::ceil(2.0 * azHalf / colStep));
  std::vector<double> colData(cols * c.samples, 0.0);
  std::vector<double> colBest(cols * c.samples, 0.0);
  std::vector<std::uint32_t> colLabel(cols * c.samples, 0);
  std::vector<long> colLo(cols, static_cast<long>(c.samples));
  std::vector<long> colHi(cols, -1);

  // Sort the hits into their columns, keeping their order within each, so
  // every thread owns whole columns and then whole beams. Nothing is shared
  // between threads, and the sums run in the same order for any number of
  // threads: the result is the same bits whatever the machine.
  std::vector<std::size_t> start(cols + 1, 0);
  std::vector<std::size_t> colOf(_hits.size(), cols);
  for (std::size_t i = 0; i < _hits.size(); ++i)
  {
    const RayHit &h = _hits[i];
    if (!h.hit || h.range <= 0.0 || std::abs(h.ray.azimuth) >= azHalf)
      continue;
    colOf[i] = std::min(cols - 1,
      static_cast<std::size_t>((h.ray.azimuth + azHalf) / colStep));
    ++start[colOf[i] + 1];
  }
  for (std::size_t col = 0; col < cols; ++col)
    start[col + 1] += start[col];
  std::vector<std::size_t> order(start[cols]);
  {
    std::vector<std::size_t> next(start.begin(), start.end() - 1);
    for (std::size_t i = 0; i < _hits.size(); ++i)
    {
      if (colOf[i] < cols)
        order[next[colOf[i]]++] = i;
    }
  }

  const std::size_t threads = c.threads > 0 ? c.threads :
    std::max(1u, std::thread::hardware_concurrency());

  // Each column: the sonar equation for every hit in it, spread over the
  // range samples its footprint and the pulse cover.
  ParallelFor(cols, threads, [&](std::size_t _col) {
    for (std::size_t n = start[_col]; n < start[_col + 1]; ++n)
    {
      const RayHit &h = _hits[order[n]];
      const double r = h.range;
      const double apparent = r * rangeScale;
      if (apparent < c.rangeMin || apparent >= c.rangeMax)
        continue;

      const double vertical = VerticalPattern(c, h.ray.elevation);
      if (vertical < kPatternFloor)
        continue;

      // Sonar equation for the patch this ray stands for.
      const double cosInc = std::max(std::cos(h.incidence), kMinCos);
      const double level = vertical * Backscatter(h.properties, h.incidence) *
        h.ray.solidAngle / (r * r * cosInc) *
        std::pow(10.0, -2.0 * this->alpha * r / 10.0);
      if (level <= 0.0)
        continue;

      // The patch's footprint on a slanted surface spans a range interval,
      // and the pulse smears it further; spread the energy uniformly over
      // the wider of the two (at least one sample, at most a few).
      // From the sampler, which knows the neighbors; otherwise estimated
      // from the incidence, and capped since a lone ray cannot tell a
      // slanted plane from the edge of a target.
      double footprint = h.footprint * rangeScale;
      if (footprint <= 0.0)
      {
        const double tanInc = std::min(std::tan(h.incidence), 50.0);
        footprint = std::min(apparent * h.ray.spacing * tanInc,
          kMaxFootprintSamples * dr);
      }
      const double width = std::max({footprint, pulseRange, dr});
      const double lo = apparent - 0.5 * width;
      const double hi = apparent + 0.5 * width;
      const long k0 = std::max<long>(0,
        static_cast<long>(std::floor((lo - c.rangeMin) / dr)));
      const long k1 = std::min<long>(static_cast<long>(c.samples) - 1,
        static_cast<long>(std::floor((hi - c.rangeMin) / dr)));

      colLo[_col] = std::min(colLo[_col], k0);
      colHi[_col] = std::max(colHi[_col], k1);
      for (long k = k0; k <= k1; ++k)
      {
        const double cellLo = c.rangeMin + static_cast<double>(k) * dr;
        const double overlap = std::min(hi, cellLo + dr) -
          std::max(lo, cellLo);
        if (overlap <= 0.0)
          continue;
        const double add = level * overlap / width;
        const std::size_t idx = _col * c.samples + static_cast<std::size_t>(k);
        colData[idx] += add;
        if (add > colBest[idx])
        {
          colBest[idx] = add;
          colLabel[idx] = h.label;
        }
      }
    }
  });

  // Each beam gathers from the columns its pattern reaches, in column order.
  ParallelFor(c.beams, threads, [&](std::size_t _j) {
    const double beamAz = out.azimuths[_j];
    const double reachAz = static_cast<double>(beamReach + 1) * beamSpacing;
    const long first = std::max<long>(0, static_cast<long>(
      std::floor((beamAz - reachAz + azHalf) / colStep)));
    const long last = std::min<long>(static_cast<long>(cols) - 1,
      static_cast<long>(std::ceil((beamAz + reachAz + azHalf) / colStep)));
    for (long col = first; col <= last; ++col)
    {
      const auto ucol = static_cast<std::size_t>(col);
      if (colHi[ucol] < colLo[ucol])
        continue;
      const double az = -azHalf + (static_cast<double>(col) + 0.5) * colStep;
      const double w = HorizontalPattern(c, az - beamAz);
      if (w < kPatternFloor)
        continue;
      for (long k = colLo[ucol]; k <= colHi[ucol]; ++k)
      {
        const std::size_t from = ucol * c.samples + static_cast<std::size_t>(k);
        const std::size_t idx = _j * c.samples + static_cast<std::size_t>(k);
        out.data[idx] += w * colData[from];
        if (w * colBest[from] > best[idx])
        {
          best[idx] = w * colBest[from];
          out.labels[idx] = colLabel[from];
        }
      }
    }
  });

  if (c.speckle)
  {
    Random rng(c.seed, _ping, kSpeckleStream);
    for (double &v : out.data)
      v *= rng.Exponential();
  }
  return out;
}
}  // namespace gz::maritime::sonar
