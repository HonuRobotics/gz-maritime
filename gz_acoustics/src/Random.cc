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
#include "gz/sim/acoustics/Random.hh"

#include <cmath>
#include <cstdint>
#include <string>

#include <gz/math/Helpers.hh>

namespace gz::sim::acoustics
{
//////////////////////////////////////////////////
std::uint32_t Hash(const std::string &_text)
{
  std::uint32_t hash{2166136261u};
  for (const unsigned char c : _text)
  {
    hash ^= c;
    hash *= 16777619u;
  }
  return hash;
}

//////////////////////////////////////////////////
double Uniform(Generator &_rng)
{
  // One 32 bit word of the generator, whose sequence the standard fixes.
  return static_cast<double>(_rng()) / 4294967296.0;
}

//////////////////////////////////////////////////
double Normal(Generator &_rng, double _mean, double _stddev)
{
  // Box and Muller; 1 - u is in (0, 1], so the logarithm is finite.
  const double u = 1.0 - Uniform(_rng);
  const double v = Uniform(_rng);
  return _mean + _stddev * std::sqrt(-2.0 * std::log(u)) *
      std::cos(2.0 * GZ_PI * v);
}

//////////////////////////////////////////////////
double ApplyNoise(const Noise &_noise, double _value, Generator &_rng)
{
  double value = _value + _noise.bias;
  if (_noise.stddev > 0.0 || _noise.mean != 0.0)
    value += Normal(_rng, _noise.mean, _noise.stddev);
  if (_noise.precision > 0.0)
    value = std::round(value / _noise.precision) * _noise.precision;
  return value;
}
}  // namespace gz::sim::acoustics
