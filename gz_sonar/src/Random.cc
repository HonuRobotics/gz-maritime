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
#include "gz/maritime/sonar/Random.hh"

#include <cmath>

namespace gz::maritime::sonar
{
//////////////////////////////////////////////////
Random::Random(std::uint64_t _seed, std::uint64_t _ping,
  std::uint64_t _stream)
  : state(_seed * 0x9E3779B97F4A7C15ULL ^ (_ping + 0x632BE59BD9B4E019ULL) *
          0xBF58476D1CE4E5B9ULL ^ (_stream + 1) * 0x94D049BB133111EBULL)
{
  // Stir, so nearby seeds start far apart.
  this->Next();
  this->Next();
}

//////////////////////////////////////////////////
std::uint64_t Random::Next()
{
  std::uint64_t z = (this->state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

//////////////////////////////////////////////////
double Random::Uniform()
{
  // 53 random bits, shifted off zero.
  return (static_cast<double>(this->Next() >> 11) + 0.5) *
    (1.0 / 9007199254740992.0);
}

//////////////////////////////////////////////////
double Random::Exponential()
{
  return -std::log(this->Uniform());
}

//////////////////////////////////////////////////
double Random::Normal()
{
  // Box-Muller, one value per call.
  const double u1 = this->Uniform();
  const double u2 = this->Uniform();
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
}
}  // namespace gz::maritime::sonar
