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
#ifndef GZ_UNDERWATER_RENDERING_TEST_OWNPARTITION_HH_
#define GZ_UNDERWATER_RENDERING_TEST_OWNPARTITION_HH_

#include <unistd.h>

#include <cstdlib>
#include <string>

namespace gz::sim::underwater_test
{
/// \brief A transport partition of this process's own, set before any
/// node is made, so copies of a test run at once do not hear each other.
inline const bool kOwnPartition = []
{
  const std::string partition =
      "gz_underwater_rendering_test_" + std::to_string(getpid());
  return setenv("GZ_PARTITION", partition.c_str(), 1) == 0;
}();
}  // namespace gz::sim::underwater_test

#endif  // GZ_UNDERWATER_RENDERING_TEST_OWNPARTITION_HH_
