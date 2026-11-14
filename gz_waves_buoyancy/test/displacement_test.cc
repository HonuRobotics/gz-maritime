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

#include <cmath>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Box.hh>
#include <sdf/Geometry.hh>

#include "gz/sim/waves_buoyancy/Displacement.hh"

using namespace gz;
using namespace sim;
using namespace waves_buoyancy;

namespace
{
/// \brief A 2 x 1 x 0.5 m box.
/// \return The shape.
BuoyantShape Box()
{
  BuoyantShape s;
  s.kind = BuoyantShape::Kind::kBox;
  s.size = math::Vector3d(2, 1, 0.5);
  return s;
}
}

//////////////////////////////////////////////////
/// Flat water through a box's middle: half its volume, centred a quarter of
/// its height down.
TEST(Displacement, FlatWaterHalfway)
{
  const auto d = Below(Box(), math::Pose3d(3, 4, 0, 0, 0, 0),
                       math::Vector3d(0, 0, 0), math::Vector3d::UnitZ);
  EXPECT_NEAR(0.5, d.volume, 1e-9);
  EXPECT_NEAR(3.0, d.centre.X(), 1e-9);
  EXPECT_NEAR(4.0, d.centre.Y(), 1e-9);
  EXPECT_NEAR(-0.125, d.centre.Z(), 1e-9);
}

//////////////////////////////////////////////////
/// A crest over the box submerges more of it, a trough less; above the
/// water nothing, under it all.
TEST(Displacement, CrestTroughOutAndUnder)
{
  const math::Pose3d at(0, 0, 0, 0, 0, 0);
  const auto up = math::Vector3d::UnitZ;
  EXPECT_NEAR(0.8, Below(Box(), at, {0, 0, 0.15}, up).volume, 1e-9);
  EXPECT_NEAR(0.2, Below(Box(), at, {0, 0, -0.15}, up).volume, 1e-9);
  EXPECT_NEAR(0.0, Below(Box(), at, {0, 0, -1.0}, up).volume, 1e-9);
  const auto under = Below(Box(), at, {0, 0, 2.0}, up);
  EXPECT_NEAR(1.0, under.volume, 1e-9);
  EXPECT_NEAR(0.0, under.centre.Length(), 1e-9);
}

//////////////////////////////////////////////////
/// A sloping surface through the box's middle keeps half its volume but
/// moves the centre of buoyancy to the side the water is deeper on, which
/// is what heels a hull on a wave's flank.
TEST(Displacement, SlopeMovesTheCentreDownhill)
{
  // The surface rises towards +x.
  const math::Vector3d normal(-std::sin(0.1), 0, std::cos(0.1));
  const auto d = Below(Box(), math::Pose3d::Zero, {0, 0, 0}, normal);
  EXPECT_NEAR(0.5, d.volume, 1e-9);
  EXPECT_GT(d.centre.X(), 0.05);
}

//////////////////////////////////////////////////
/// The surface is a world plane: a box turned about the vertical is cut by
/// it the same, in world terms.
TEST(Displacement, WorldPlaneWhateverTheShapesHeading)
{
  const auto flat = Below(Box(), math::Pose3d(0, 0, 0, 0, 0, 0),
                          {0, 0, 0.1}, math::Vector3d::UnitZ);
  const auto turned = Below(Box(), math::Pose3d(0, 0, 0, 0, 0, GZ_PI / 2),
                            {0, 0, 0.1}, math::Vector3d::UnitZ);
  EXPECT_NEAR(flat.volume, turned.volume, 1e-9);
  EXPECT_NEAR(flat.centre.Z(), turned.centre.Z(), 1e-9);
}

//////////////////////////////////////////////////
/// A sphere and a cylinder, half under.
TEST(Displacement, SphereAndCylinder)
{
  BuoyantShape sphere;
  sphere.kind = BuoyantShape::Kind::kSphere;
  sphere.radius = 0.5;
  const auto s = Below(sphere, math::Pose3d::Zero, {0, 0, 0},
                       math::Vector3d::UnitZ);
  EXPECT_NEAR(sphere.Volume() / 2, s.volume, 1e-6);
  EXPECT_NEAR(-3.0 / 8.0 * 0.5, s.centre.Z(), 1e-6);

  BuoyantShape cylinder;
  cylinder.kind = BuoyantShape::Kind::kCylinder;
  cylinder.radius = 0.2;
  cylinder.length = 1.0;
  const auto c = Below(cylinder, math::Pose3d::Zero, {0, 0, 0},
                       math::Vector3d::UnitZ);
  EXPECT_NEAR(cylinder.Volume() / 2, c.volume, 1e-6);
}

//////////////////////////////////////////////////
/// A geometry becomes a shape when it is one the buoyancy floats.
TEST(Displacement, ShapeFromGeometry)
{
  sdf::Geometry box;
  box.SetType(sdf::GeometryType::BOX);
  sdf::Box b;
  b.SetSize(math::Vector3d(1, 2, 3));
  box.SetBoxShape(b);
  const auto shape = ShapeFromGeometry(box, math::Pose3d(0, 0, 1, 0, 0, 0));
  ASSERT_TRUE(shape.has_value());
  EXPECT_EQ(BuoyantShape::Kind::kBox, shape->kind);
  EXPECT_DOUBLE_EQ(6.0, shape->Volume());
  EXPECT_DOUBLE_EQ(1.0, shape->pose.Pos().Z());

  sdf::Geometry mesh;
  mesh.SetType(sdf::GeometryType::MESH);
  EXPECT_FALSE(ShapeFromGeometry(mesh, math::Pose3d::Zero).has_value());
}
