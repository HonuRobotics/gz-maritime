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
/// \brief The random draws of the acoustics' rules, the dropouts and the
/// outliers, from a generator of their own: nothing here touches gz-math's
/// global generator, so a seed for the acoustics changes no other sensor or
/// system. Every draw is written out here, not left to the standard
/// library's distributions, so a seed gives the same numbers on every
/// standard library. A receiver's noise is gz-sensors', as on any sensor.

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
}  // namespace gz::sim::acoustics

#endif  // GZ_SIM_ACOUSTICS_RANDOM_HH_
