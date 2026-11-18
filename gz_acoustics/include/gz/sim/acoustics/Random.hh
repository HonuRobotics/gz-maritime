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
#ifndef GZ_SIM_ACOUSTICS_RANDOM_HH_
#define GZ_SIM_ACOUSTICS_RANDOM_HH_

#include <cstdint>
#include <random>
#include <string>

/// \file
/// \brief The random draws of the acoustics, from a generator of their own:
/// nothing here touches gz-math's global generator, so a seed for the
/// acoustics changes no other sensor or system. Every draw is written out
/// here, not left to the standard library's distributions, so a seed gives
/// the same numbers on every standard library.

namespace gz::sim::acoustics
{
/// \brief The generator the acoustics draw from.
using Generator = std::mt19937;

/// \brief A 32 bit hash of a string, FNV-1a: the same everywhere, which
/// std::hash is not.
/// \param[in] _text The string.
/// \return Its hash.
std::uint32_t Hash(const std::string &_text);

/// \brief A number in [0, 1).
/// \param[in,out] _rng The generator.
/// \return The number.
double Uniform(Generator &_rng);

/// \brief A number from a normal distribution.
/// \param[in,out] _rng The generator.
/// \param[in] _mean Its mean.
/// \param[in] _stddev Its standard deviation.
/// \return The number.
double Normal(Generator &_rng, double _mean, double _stddev);

/// \brief The noise of one measured channel, as a Gazebo sensor's gaussian
/// `<noise>` block sets it.
struct Noise
{
  /// \brief Mean of the noise added to every reading.
  double mean{0.0};

  /// \brief Standard deviation of the noise added to every reading.
  double stddev{0.0};

  /// \brief A constant added to every reading, drawn once.
  double bias{0.0};

  /// \brief Readings are rounded to a multiple of this; 0 does not round.
  double precision{0.0};
};

/// \brief A value with a channel's noise.
/// \param[in] _noise The noise.
/// \param[in] _value The true value.
/// \param[in,out] _rng The generator.
/// \return The noisy value.
double ApplyNoise(const Noise &_noise, double _value, Generator &_rng);
}  // namespace gz::sim::acoustics

#endif  // GZ_SIM_ACOUSTICS_RANDOM_HH_
