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
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "CaptureWorld.hh"

using gz::sim::underwater_test::Capture;
using gz::sim::underwater_test::CaptureWorld;

//////////////////////////////////////////////////
// When the renderer cannot draw the water, here because its pieces are not
// where GZ_UNDERWATER_HLMS_DIR says, it does not mark the world: the
// reference camera draws the water, as in a world without the GPU path.
// A process of its own: once installed, the renderer stays installed for
// the rest of the process.
TEST(UnderwaterRenderingFallback, ReferenceCameraDrawsTheWater)
{
  const Capture off = CaptureWorld("gpu_off.sdf");
  if (!off.rendered)
    GTEST_SKIP() << "nothing rendered: no GPU?";
  const Capture failed = CaptureWorld("gpu_on.sdf");

  ASSERT_FALSE(off.underWater.empty());
  ASSERT_FALSE(failed.underWater.empty());
  EXPECT_EQ(failed.underImage, off.underImage) << "the renderer drew water";

  // The reference camera's water, as without the GPU path; it processes a
  // frame with the pose of the latest step, so to a step or so.
  ASSERT_EQ(failed.underWater.size(), off.underWater.size());
  std::vector<int> diffs;
  for (std::size_t i = 0; i < off.underWater.size(); ++i)
  {
    diffs.push_back(std::abs(
        static_cast<int>(static_cast<std::uint8_t>(failed.underWater[i])) -
        static_cast<int>(static_cast<std::uint8_t>(off.underWater[i]))));
  }
  double mean = 0;
  for (int d : diffs)
    mean += d;
  mean /= diffs.size();
  std::sort(diffs.begin(), diffs.end());
  std::cout << "reference camera against the run without the GPU path: mean "
            << mean << ", 99th percentile " << diffs[diffs.size() * 99 / 100]
            << ", worst " << diffs.back() << " steps\n";
  EXPECT_LE(mean, 1.0);
  EXPECT_LE(diffs[diffs.size() * 99 / 100], 3);
  // And it is the water, not the image passed through
  EXPECT_NE(failed.underWater, failed.underImage)
      << "the reference camera did not draw the water";
}
