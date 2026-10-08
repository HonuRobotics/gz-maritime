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

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "gz/sim/marked_shapes/MarkedShapes.hh"

using namespace gz;
using namespace sim;
using marked_shapes::Cut;
using marked_shapes::Side;

namespace
{
/// \brief Half extents of a 1 m cube.
const math::Vector3d kHalf(0.5, 0.5, 0.5);
}  // namespace

/////////////////////////////////////////////////
/// A cube straddling the water splits in two halves, each centred in its
/// own part.
TEST(Cut, HalfAboveHalfBelow)
{
  const math::Pose3d pose(3, 4, 0, 0, 0, 0);
  const auto above = Cut(pose, kHalf, 0.0, Side::kAbove);
  const auto below = Cut(pose, kHalf, 0.0, Side::kBelow);
  ASSERT_TRUE(above.any);
  ASSERT_TRUE(below.any);
  EXPECT_DOUBLE_EQ(0.5, above.fraction);
  EXPECT_DOUBLE_EQ(0.5, below.fraction);
  EXPECT_EQ(math::Vector3d(3, 4, 0.25), above.centre);
  EXPECT_EQ(math::Vector3d(3, 4, -0.25), below.centre);
}

/////////////////////////////////////////////////
/// A cube clear of the water is all on one side and none on the other.
TEST(Cut, AllOnOneSide)
{
  const math::Pose3d deep(0, 0, -2, 0, 0, 0);
  EXPECT_FALSE(Cut(deep, kHalf, 0.0, Side::kAbove).any);
  const auto wet = Cut(deep, kHalf, 0.0, Side::kBelow);
  ASSERT_TRUE(wet.any);
  EXPECT_DOUBLE_EQ(1.0, wet.fraction);
  EXPECT_EQ(math::Vector3d(0, 0, -2), wet.centre);

  const math::Pose3d high(0, 0, 2, 0, 0, 0);
  EXPECT_FALSE(Cut(high, kHalf, 0.0, Side::kBelow).any);
  EXPECT_DOUBLE_EQ(1.0, Cut(high, kHalf, 0.0, Side::kAbove).fraction);
}

/////////////////////////////////////////////////
/// The water level moves the cut.
TEST(Cut, WaterLevel)
{
  const math::Pose3d pose(0, 0, 1.25, 0, 0, 0);
  const auto below = Cut(pose, kHalf, 1.5, Side::kBelow);
  ASSERT_TRUE(below.any);
  EXPECT_DOUBLE_EQ(0.75, below.fraction);
  EXPECT_DOUBLE_EQ(1.125, below.centre.Z());
}

/////////////////////////////////////////////////
/// A tilted shape is cut by its bounding box in the world: a cube turned
/// 45 degrees about x reaches sqrt(2) / 2 above and below its centre.
TEST(Cut, TiltedShape)
{
  const math::Pose3d pose(0, 0, 0.5, GZ_PI / 4, 0, 0);
  const auto below = Cut(pose, kHalf, 0.0, Side::kBelow);
  ASSERT_TRUE(below.any);
  const double hz = std::sqrt(2.0) / 2.0;
  EXPECT_NEAR((hz - 0.5) / (2 * hz), below.fraction, 1e-12);
}
