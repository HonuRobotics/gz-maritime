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

/// \brief A footprint spreads over at most this many samples.
constexpr double kMaxFootprintSamples = 64.0;

double Sinc2(double _x)
{
  if (std::abs(_x) < 1e-9)
    return 1.0;
  const double s = std::sin(M_PI * _x) / (M_PI * _x);
  return s * s;
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
  // column is then spread into the beams once. Same sums, far fewer of them.
  const double colStep = std::min(beamSpacing, c.beamWidth) /
    static_cast<double>(std::max<std::size_t>(c.raysPerBeam, 1));
  const double azHalf = 0.5 * c.horizontalFov + 2.0 * c.beamWidth;
  const auto cols = static_cast<std::size_t>(std::ceil(2.0 * azHalf / colStep));
  std::vector<double> colData(cols * c.samples, 0.0);
  std::vector<double> colBest(cols * c.samples, 0.0);
  std::vector<std::uint32_t> colLabel(cols * c.samples, 0);
  std::vector<long> colLo(cols, static_cast<long>(c.samples));
  std::vector<long> colHi(cols, -1);

  for (const RayHit &h : _hits)
  {
    if (!h.hit || h.range <= 0.0)
      continue;
    const double r = h.range;
    const double apparent = r * rangeScale;
    if (apparent < c.rangeMin || apparent >= c.rangeMax)
      continue;
    if (std::abs(h.ray.azimuth) >= azHalf)
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
    // and the pulse smears it further; spread the energy uniformly over the
    // wider of the two (at least one sample).
    const double tanInc = std::min(std::tan(h.incidence), 50.0);
    const double footprint = std::min(apparent * h.ray.spacing * tanInc,
      kMaxFootprintSamples * dr);
    const double width = std::max({footprint, pulseRange, dr});
    const double lo = apparent - 0.5 * width;
    const double hi = apparent + 0.5 * width;
    const long k0 = std::max<long>(0,
      static_cast<long>(std::floor((lo - c.rangeMin) / dr)));
    const long k1 = std::min<long>(static_cast<long>(c.samples) - 1,
      static_cast<long>(std::floor((hi - c.rangeMin) / dr)));

    const auto col = std::min(cols - 1, static_cast<std::size_t>(
      (h.ray.azimuth + azHalf) / colStep));
    colLo[col] = std::min(colLo[col], k0);
    colHi[col] = std::max(colHi[col], k1);
    for (long k = k0; k <= k1; ++k)
    {
      const double cellLo = c.rangeMin + static_cast<double>(k) * dr;
      const double overlap = std::min(hi, cellLo + dr) - std::max(lo, cellLo);
      if (overlap <= 0.0)
        continue;
      const double add = level * overlap / width;
      const std::size_t idx = col * c.samples + static_cast<std::size_t>(k);
      colData[idx] += add;
      if (add > colBest[idx])
      {
        colBest[idx] = add;
        colLabel[idx] = h.label;
      }
    }
  }

  for (std::size_t col = 0; col < cols; ++col)
  {
    if (colHi[col] < colLo[col])
      continue;
    const double az = -azHalf + (static_cast<double>(col) + 0.5) * colStep;
    const double jCenter = (az + 0.5 * c.horizontalFov) / beamSpacing - 0.5;
    const long j0 = std::max<long>(0,
      static_cast<long>(std::floor(jCenter)) - beamReach);
    const long j1 = std::min<long>(static_cast<long>(c.beams) - 1,
      static_cast<long>(std::ceil(jCenter)) + beamReach);
    for (long j = j0; j <= j1; ++j)
    {
      const double w = HorizontalPattern(c,
        az - out.azimuths[static_cast<std::size_t>(j)]);
      if (w < kPatternFloor)
        continue;
      for (long k = colLo[col]; k <= colHi[col]; ++k)
      {
        const std::size_t from = col * c.samples + static_cast<std::size_t>(k);
        const std::size_t idx =
          static_cast<std::size_t>(j) * c.samples + static_cast<std::size_t>(k);
        out.data[idx] += w * colData[from];
        if (w * colBest[from] > best[idx])
        {
          best[idx] = w * colBest[from];
          out.labels[idx] = colLabel[from];
        }
      }
    }
  }

  if (c.speckle)
  {
    Random rng(c.seed, _ping, kSpeckleStream);
    for (double &v : out.data)
      v *= rng.Exponential();
  }
  return out;
}
}  // namespace gz::maritime::sonar
