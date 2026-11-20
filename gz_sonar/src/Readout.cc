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
#include "gz/maritime/sonar/Readout.hh"

#include <algorithm>
#include <cmath>
#include <limits>

#include "gz/maritime/sonar/Random.hh"

namespace gz::maritime::sonar
{
namespace
{
constexpr std::uint64_t kNoiseStream = 2;
constexpr std::uint64_t kDetectionStream = 3;

/// \brief Automatic gain puts this fraction of the lit cells below full
/// scale.
constexpr double kAutoGainQuantile = 0.995;
}  // namespace

double TimeVariedGain(const SonarConfig &_config, double _alpha,
  double _range)
{
  if (_config.tvg == Tvg::kNone)
    return 0.0;
  double g = _config.tvgSpreading * std::log10(std::max(_range, 1e-3));
  if (_config.tvg == Tvg::kSpreadingAbsorption)
    g += 2.0 * _alpha * _range;
  return g;
}

double GainTracker::Update(double _pingGain, double _dt, double _timeConstant)
{
  if (!this->started || _timeConstant <= 0.0)
  {
    this->started = true;
    this->gain = _pingGain;
    return this->gain;
  }
  const double a = 1.0 - std::exp(-std::max(_dt, 0.0) / _timeConstant);
  this->gain += a * (_pingGain - this->gain);
  return this->gain;
}

Levels ReadLevels(const BeamIntensity &_intensity, const SonarConfig &_config,
  double _alpha, std::uint64_t _ping, GainTracker *_tracker, double _dt)
{
  Levels out;
  out.beams = _intensity.beams;
  out.samples = _intensity.samples;
  out.ranges = _intensity.ranges;
  out.azimuths = _intensity.azimuths;
  for (double r : out.ranges)
    out.tvgDb.push_back(TimeVariedGain(_config, _alpha, r));

  // Level plus time-varied gain, before gain.
  const double off = -std::numeric_limits<double>::infinity();
  std::vector<double> raw(_intensity.data.size(), off);
  std::vector<double> lit;
  for (std::size_t j = 0; j < out.beams; ++j)
  {
    for (std::size_t k = 0; k < out.samples; ++k)
    {
      const double v = _intensity.At(j, k);
      if (v <= 0.0)
        continue;
      const double d = 10.0 * std::log10(v) + out.tvgDb[k];
      raw[j * out.samples + k] = d;
      lit.push_back(d);
    }
  }

  if (_config.gainDb)
  {
    out.gainDb = *_config.gainDb;
  }
  else if (!lit.empty())
  {
    const auto n = static_cast<std::size_t>(
      kAutoGainQuantile * static_cast<double>(lit.size() - 1));
    std::nth_element(lit.begin(), lit.begin() + static_cast<long>(n),
      lit.end());
    out.gainDb = -lit[n];
    if (_tracker)
      out.gainDb = _tracker->Update(out.gainDb, _dt, _config.gainTimeConstant);
  }

  Random rng(_config.seed, _ping, kNoiseStream);
  const double noise = std::pow(10.0, _config.noiseFloorDb / 10.0);
  out.db.resize(raw.size());
  for (std::size_t i = 0; i < raw.size(); ++i)
  {
    double lin = std::isfinite(raw[i]) ?
      std::pow(10.0, (raw[i] + out.gainDb) / 10.0) : 0.0;
    lin += noise * rng.Exponential();
    out.db[i] = 10.0 * std::log10(lin);
  }
  return out;
}

ImagingFrame ReadImaging(const Levels &_levels, const SonarConfig &_config)
{
  ImagingFrame out;
  out.beams = _levels.beams;
  out.samples = _levels.samples;
  out.ranges = _levels.ranges;
  out.azimuths = _levels.azimuths;
  out.gainDb = _levels.gainDb;
  out.data.resize(_levels.db.size());
  const double span = _config.dynamicRangeDb;
  for (std::size_t i = 0; i < _levels.db.size(); ++i)
  {
    const double d = _levels.db[i];
    double v = 0.0;
    if (d >= -span)
      v = 1.0 + 254.0 * std::min(1.0, (d + span) / span);
    out.data[i] = static_cast<std::uint8_t>(std::lround(v));
  }
  return out;
}

SidescanLine ReadSidescan(const Levels &_levels)
{
  SidescanLine out;
  out.ranges = _levels.ranges;
  out.tvgDb = _levels.tvgDb;
  out.gainDb = _levels.gainDb;
  out.db.reserve(_levels.samples);
  for (std::size_t k = 0; k < _levels.samples; ++k)
    out.db.push_back(static_cast<float>(_levels.db[k]));
  return out;
}

std::vector<Sounding> ReadProfiling(const Levels &_levels,
  const SonarConfig &_config, double _assumedSoundSpeed, std::uint64_t _ping)
{
  const double threshold = _config.noiseFloorDb +
    _config.detectionThresholdDb;
  const double dr = _levels.ranges.size() > 1 ?
    _levels.ranges[1] - _levels.ranges[0] : 0.0;
  const double rangeNoise = _config.rangeNoise ? *_config.rangeNoise :
    0.5 * dr;
  Random rng(_config.seed, _ping, kDetectionStream);

  // The strongest sample of each beam.
  struct Peak
  {
    std::size_t beam;
    std::size_t sample;
    double db;
  };
  std::vector<Peak> peaks;
  for (std::size_t j = 0; j < _levels.beams; ++j)
  {
    std::size_t bestK = 0;
    double bestDb = -std::numeric_limits<double>::infinity();
    for (std::size_t k = 0; k < _levels.samples; ++k)
    {
      const double d = _levels.db[j * _levels.samples + k];
      if (d > bestDb)
      {
        bestDb = d;
        bestK = k;
      }
    }
    if (bestDb > threshold)
      peaks.push_back({j, bestK, bestDb});
  }

  auto makeSounding = [&](const Peak &_p) {
    Sounding s;
    s.angle = _levels.azimuths[_p.beam];
    const double snr = _p.db - threshold;
    const double sigma = rangeNoise * (1.0 + std::pow(10.0, -snr / 20.0));
    s.range = _levels.ranges[_p.sample] + sigma * rng.Normal();
    s.travelTime = 2.0 * s.range / _assumedSoundSpeed;
    s.levelDb = _p.db - _config.noiseFloorDb;
    return s;
  };

  std::vector<Sounding> out;
  if (_config.readout == ProfilingReadout::kBeamformed)
  {
    for (const Peak &p : peaks)
      out.push_back(makeSounding(p));

    // A sounding far from the median of its neighbors is an outlier.
    const double tolerance = std::max(3.0 * rangeNoise, 5.0 * dr);
    for (std::size_t i = 0; i < out.size(); ++i)
    {
      std::vector<double> near;
      for (std::size_t m = (i >= 2 ? i - 2 : 0);
           m <= std::min(out.size() - 1, i + 2); ++m)
      {
        if (m != i)
          near.push_back(out[m].range);
      }
      if (near.size() < 2)
        continue;
      std::nth_element(near.begin(),
        near.begin() + static_cast<long>(near.size() / 2), near.end());
      if (std::abs(out[i].range - near[near.size() / 2]) > tolerance)
        out[i].flag = SoundingFlag::kBadFilter;
    }
    return out;
  }

  // Angle of arrival: the fan is split into maxSoundings equal sectors and
  // each reports its strongest echo, so the soundings spread across the
  // swath as the device's do, rather than crowding where echoes are
  // strongest.
  std::vector<Peak> chosen;
  const std::size_t sectors = std::max<std::size_t>(_config.maxSoundings, 1);
  for (std::size_t s = 0; s < sectors; ++s)
  {
    const std::size_t lo = s * _levels.beams / sectors;
    const std::size_t hi = (s + 1) * _levels.beams / sectors;
    const Peak *best = nullptr;
    for (const Peak &p : peaks)
    {
      if (p.beam >= lo && p.beam < hi && (!best || p.db > best->db))
        best = &p;
    }
    if (best)
      chosen.push_back(*best);
  }
  for (const Peak &p : chosen)
  {
    Sounding s = makeSounding(p);
    s.angle += _config.angleNoise * rng.Normal();
    s.levelDb = std::numeric_limits<double>::quiet_NaN();
    out.push_back(s);
  }
  return out;
}
}  // namespace gz::maritime::sonar
