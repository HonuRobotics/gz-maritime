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
#include "gz/sim/acoustics/Hearing.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <gz/math/Helpers.hh>

namespace gz::sim::acoustics
{
//////////////////////////////////////////////////
Duration FromSeconds(double _seconds)
{
  return std::chrono::duration_cast<Duration>(
      std::chrono::nanoseconds(std::llround(_seconds * 1e9)));
}

//////////////////////////////////////////////////
Duration FirstWindowStart(const Duration &_now, const Duration &_dt)
{
  const Duration start = _now - _dt;
  // The window's start is exclusive: one tick before zero takes a ping at
  // time zero in.
  if (start <= Duration::zero())
    return -Duration(1);
  return start;
}

//////////////////////////////////////////////////
std::vector<Duration> EmissionTimes(const PingerData &_pinger,
    const Duration &_after, const Duration &_upTo)
{
  std::vector<Duration> times;
  if (!_pinger.active || !(_pinger.period > 0.0) || _upTo <= _after)
    return times;

  // In whole nanoseconds, so a ping on a step boundary is counted once.
  const Duration period = FromSeconds(_pinger.period);
  const Duration offset = FromSeconds(_pinger.offset);
  if (period <= Duration::zero())
    return times;

  // The first k with offset + k period > after, and k >= 0.
  std::int64_t k = 0;
  if (_after >= offset)
    k = (_after - offset) / period + 1;
  for (Duration t = offset + k * period; t <= _upTo; t += period)
    times.push_back(t);
  return times;
}

//////////////////////////////////////////////////
Direction Measure(const math::Pose3d &_receiver,
    const math::Vector3d &_source)
{
  const math::Vector3d v =
      _receiver.Rot().RotateVectorReverse(_source - _receiver.Pos());
  Direction d;
  d.range = v.Length();
  d.bearing = std::atan2(v.Y(), v.X());
  d.elevation = std::atan2(v.Z(), std::hypot(v.X(), v.Y()));
  return d;
}

//////////////////////////////////////////////////
bool InBand(double _frequency, double _centre, double _bandwidth)
{
  if (_centre <= 0.0)
    return true;
  return std::abs(_frequency - _centre) <= 0.5 * std::max(_bandwidth, 0.0);
}

//////////////////////////////////////////////////
bool InElevation(double _elevation, double _min, double _max)
{
  return _elevation >= _min && _elevation <= _max;
}

//////////////////////////////////////////////////
double DropoutProbability(double _distance, double _reach, double _start)
{
  if (_reach <= 0.0 || _start >= 1.0)
    return 0.0;
  const double start = std::max(_start, 0.0);
  const double fraction = _distance / _reach;
  if (fraction <= start)
    return 0.0;
  return std::clamp((fraction - start) / (1.0 - start), 0.0, 1.0);
}

//////////////////////////////////////////////////
double Amplitude(double _distance)
{
  return -20.0 * std::log10(std::max(_distance, 1.0));
}

//////////////////////////////////////////////////
Duration TravelTime(double _distance, double _speedOfSound)
{
  if (_speedOfSound <= 0.0)
    return Duration::zero();
  return FromSeconds(_distance / _speedOfSound);
}

//////////////////////////////////////////////////
bool Overlap(const Duration &_a, const Duration &_aPulse,
    const Duration &_b, const Duration &_bPulse)
{
  if (_a <= _b)
    return _b - _a < _aPulse;
  return _a - _b < _bPulse;
}

//////////////////////////////////////////////////
double WrapAngle(double _angle)
{
  double a = std::remainder(_angle, 2.0 * GZ_PI);
  if (a <= -GZ_PI)
    a += 2.0 * GZ_PI;
  return a;
}

//////////////////////////////////////////////////
bool HidesRange(double _stddev, double _biasStdDev,
                double _dynamicBiasStdDev,
                double _dynamicBiasCorrelationTime)
{
  return _stddev > 0.0 || _biasStdDev > 0.0 ||
      (_dynamicBiasStdDev > 0.0 && _dynamicBiasCorrelationTime > 0.0);
}
}  // namespace gz::sim::acoustics
