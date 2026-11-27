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

#include <cstdint>
#include <string>

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
}  // namespace gz::sim::acoustics
