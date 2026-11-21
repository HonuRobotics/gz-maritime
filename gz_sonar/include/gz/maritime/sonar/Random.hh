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
#ifndef GZ_MARITIME_SONAR_RANDOM_HH_
#define GZ_MARITIME_SONAR_RANDOM_HH_

#include <cstdint>

namespace gz::maritime::sonar
{
/// \brief A small random generator whose sequence is fixed by its seed on
/// every platform and compiler: splitmix64 for the state, and the
/// distributions written out here rather than taken from <random>, whose
/// distributions differ between standard libraries.
class Random
{
  /// \brief Seed from the sonar's seed, the ping number and a stream, so
  /// each stage of a ping draws its own sequence.
  public: Random(std::uint64_t _seed, std::uint64_t _ping,
              std::uint64_t _stream);

  /// \brief Uniform on (0, 1).
  public: double Uniform();

  /// \brief Exponential with mean 1: fully developed speckle in intensity.
  public: double Exponential();

  /// \brief Standard normal.
  public: double Normal();

  private: std::uint64_t Next();

  private: std::uint64_t state;
};
}  // namespace gz::maritime::sonar

#endif  // GZ_MARITIME_SONAR_RANDOM_HH_
