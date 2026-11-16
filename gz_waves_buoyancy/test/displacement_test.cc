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

#include "Displacement.hh"

using namespace gz;
using namespace sim;
using namespace waves_buoyancy;

namespace
{
/// \brief A marked shape.
/// \param[in] _type Its geometry type.
/// \param[in] _half Its half extents.
/// \return The shape.
marked_shapes::Shape Make(sdf::GeometryType _type,
                          const math::Vector3d &_half)
{
  marked_shapes::Shape s;
  s.type = _type;
  s.half = _half;
  return s;
}

/// \brief A 2 x 1 x 0.5 m box.
/// \return The shape.
marked_shapes::Shape Box()
{
  return Make(sdf::GeometryType::BOX, {1, 0.5, 0.25});
}

/// \brief A level water surface.
/// \param[in] _z Its height.
/// \return The surface.
Surface Level(double _z)
{
  Surface s;
  s.point = math::Vector3d(0, 0, _z);
  return s;
}
}

//////////////////////////////////////////////////
/// Flat water through a box's middle: half its volume, centred a quarter of
/// its height down; the other half in the air, a quarter up.
TEST(Displacement, FlatWaterHalfway)
{
  const auto d = Below(Box(), math::Pose3d(3, 4, 0, 0, 0, 0), Level(0));
  EXPECT_NEAR(0.5, d.volume, 1e-9);
  EXPECT_NEAR(3.0, d.centre.X(), 1e-9);
  EXPECT_NEAR(4.0, d.centre.Y(), 1e-9);
  EXPECT_NEAR(-0.125, d.centre.Z(), 1e-9);
  EXPECT_NEAR(0.5, d.airVolume, 1e-9);
  EXPECT_NEAR(0.125, d.airCentre.Z(), 1e-9);
}

//////////////////////////////////////////////////
/// A crest over the box submerges more of it, a trough less; above the
/// water nothing, under it all.
TEST(Displacement, CrestTroughOutAndUnder)
{
  const math::Pose3d at(0, 0, 0, 0, 0, 0);
  EXPECT_NEAR(0.8, Below(Box(), at, Level(0.15)).volume, 1e-9);
  EXPECT_NEAR(0.2, Below(Box(), at, Level(-0.15)).volume, 1e-9);
  const auto out = Below(Box(), at, Level(-1.0));
  EXPECT_NEAR(0.0, out.volume, 1e-9);
  EXPECT_NEAR(1.0, out.airVolume, 1e-9);
  EXPECT_NEAR(0.0, out.airCentre.Length(), 1e-9);
  const auto under = Below(Box(), at, Level(2.0));
  EXPECT_NEAR(1.0, under.volume, 1e-9);
  EXPECT_NEAR(0.0, under.airVolume, 1e-9);
  EXPECT_NEAR(0.0, under.centre.Length(), 1e-9);
}

//////////////////////////////////////////////////
/// A sloping surface through the box's middle keeps half its volume but
/// moves the centre of buoyancy to the side the water is deeper on, which
/// is what heels a hull on a wave's flank.
TEST(Displacement, SlopeMovesTheCentreDownhill)
{
  // The surface rises towards +x.
  Surface s;
  s.normal = math::Vector3d(-std::sin(0.1), 0, std::cos(0.1));
  const auto d = Below(Box(), math::Pose3d::Zero, s);
  EXPECT_NEAR(0.5, d.volume, 1e-9);
  EXPECT_GT(d.centre.X(), 0.05);
  EXPECT_LT(d.airCentre.X(), -0.05);
}

//////////////////////////////////////////////////
/// The surface is a world plane: a box turned about the vertical is cut by
/// it the same, in world terms.
TEST(Displacement, WorldPlaneWhateverTheShapesHeading)
{
  const auto flat = Below(Box(), math::Pose3d(0, 0, 0, 0, 0, 0), Level(0.1));
  const auto turned = Below(Box(), math::Pose3d(0, 0, 0, 0, 0, GZ_PI / 2),
                            Level(0.1));
  EXPECT_NEAR(flat.volume, turned.volume, 1e-9);
  EXPECT_NEAR(flat.centre.Z(), turned.centre.Z(), 1e-9);
}

//////////////////////////////////////////////////
/// Every shape the flat buoyancy floats floats here, half under when the
/// water is through its middle; a mesh does not.
TEST(Displacement, EveryShapeTheFlatBuoyancyFloats)
{
  struct Case
  {
    public: sdf::GeometryType type;
    public: math::Vector3d half;
    public: double volume;
  };
  const double pi = GZ_PI;
  const Case cases[] = {
    {sdf::GeometryType::BOX, {1, 0.5, 0.25}, 1.0},
    {sdf::GeometryType::SPHERE, {0.5, 0.5, 0.5}, 4.0 / 3.0 * pi / 8.0},
    {sdf::GeometryType::CYLINDER, {0.2, 0.2, 0.5}, pi * 0.04},
    // Length 1 plus a 0.2 m radius each end.
    {sdf::GeometryType::CAPSULE, {0.2, 0.2, 0.7},
     pi * 0.04 + 4.0 / 3.0 * pi * 0.008},
    {sdf::GeometryType::ELLIPSOID, {0.3, 0.4, 0.5},
     4.0 / 3.0 * pi * 0.3 * 0.4 * 0.5},
  };
  for (const auto &c : cases)
  {
    SCOPED_TRACE(static_cast<int>(c.type));
    const auto shape = Make(c.type, c.half);
    EXPECT_TRUE(Floats(c.type));
    EXPECT_NEAR(c.volume, Volume(shape), 1e-9);
    const auto d = Below(shape, math::Pose3d::Zero, Level(0));
    EXPECT_NEAR(c.volume / 2, d.volume, 1e-6);
    EXPECT_NEAR(c.volume / 2, d.airVolume, 1e-6);
    EXPECT_LT(d.centre.Z(), 0.0);
    EXPECT_GT(d.airCentre.Z(), 0.0);
  }
  EXPECT_FALSE(Floats(sdf::GeometryType::MESH));
  EXPECT_DOUBLE_EQ(0.0, Volume(Make(sdf::GeometryType::MESH, {1, 1, 1})));
}

//////////////////////////////////////////////////
/// A cone's centre is not its origin: in the air, the whole cone's weight
/// of air acts at its centroid, a quarter of the way up from the base.
TEST(Displacement, ConeInTheAirActsAtItsCentroid)
{
  const auto cone = Make(sdf::GeometryType::CONE, {0.5, 0.5, 1.0});
  EXPECT_NEAR(GZ_PI * 0.25 * 2.0 / 3.0, Volume(cone), 1e-9);
  const auto d = Below(cone, math::Pose3d::Zero, Level(-5.0));
  EXPECT_NEAR(0.0, d.volume, 1e-12);
  EXPECT_NEAR(Volume(cone), d.airVolume, 1e-9);
  // Base at -1, apex at +1: the centroid is at -1 + 2 / 4.
  EXPECT_NEAR(-0.5, d.airCentre.Z(), 1e-6);
}

//////////////////////////////////////////////////
/// The fitted plane on still water is the water.
TEST(FitSurface, Flat)
{
  const auto s = FitSurface(Box(), math::Pose3d(1, 2, 0, 0, 0, 0.3),
      [](double, double) { return 0.4; });
  EXPECT_NEAR(0.4, s.point.Z(), 1e-12);
  EXPECT_NEAR(1.0, s.point.X(), 1e-12);
  EXPECT_NEAR(2.0, s.point.Y(), 1e-12);
  EXPECT_NEAR(1.0, s.normal.Z(), 1e-12);
}

//////////////////////////////////////////////////
/// On a uniform slope the fitted plane is the slope, whatever the heading:
/// its normal leans downhill.
TEST(FitSurface, Slope)
{
  for (double yaw : {0.0, 0.7, GZ_PI / 2})
  {
    SCOPED_TRACE(yaw);
    const auto s = FitSurface(Box(), math::Pose3d(0, 0, 0, 0, 0, yaw),
        [](double _x, double _y) { return 0.1 * _x - 0.05 * _y; });
    const math::Vector3d expected =
        math::Vector3d(-0.1, 0.05, 1.0).Normalized();
    EXPECT_NEAR(expected.X(), s.normal.X(), 1e-9);
    EXPECT_NEAR(expected.Y(), s.normal.Y(), 1e-9);
    EXPECT_NEAR(0.0, s.point.Z(), 1e-12);
  }
}

//////////////////////////////////////////////////
/// A 4 m hull over a 4 m ripple, centred where the ripple is steepest: the
/// slope at its centre alone would tilt it 9 degrees, the plane through its
/// ends and centre averages the ripple out and stays level.
TEST(FitSurface, AveragesChopAsLongAsTheHull)
{
  const auto hull = Make(sdf::GeometryType::BOX, {2.0, 0.25, 0.25});
  const double k = 2.0 * GZ_PI / 4.0;
  const auto s = FitSurface(hull, math::Pose3d(1, 0, 0, 0, 0, 0),
      [k](double _x, double) { return -0.1 * std::cos(k * _x); });
  EXPECT_NEAR(1.0, s.normal.Z(), 1e-9);
  EXPECT_NEAR(0.0, s.point.Z(), 1e-9);
}
