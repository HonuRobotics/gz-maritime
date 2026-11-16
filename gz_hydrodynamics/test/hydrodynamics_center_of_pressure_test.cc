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

// The centre of pressure of the vendored hydrodynamics: a body pushed where
// its model is written stays level, and one whose model is left at a link
// origin off the thrust line pitches, as upstream's does.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief Push along +x, at the centre of mass [N].
constexpr double kThrust{10.0};

/// \brief Quadratic surge damping of both boxes [kg/m].
constexpr double kDamping{40.0};

/// \brief The link of a model.
/// \param[in] _ecm The entity component manager.
/// \param[in] _model Model name.
/// \return Its link named "link".
Link LinkOf(const EntityComponentManager &_ecm, const std::string &_model)
{
  const Entity model = _ecm.EntityByComponents(components::Model(),
                                               components::Name(_model));
  return Link(Model(model).LinkByName(_ecm, "link"));
}

/// \brief The angle a rotation turns by, which does not wrap as its Euler
/// angles do.
/// \param[in] _rot The rotation.
/// \return Its angle [rad], in [0, pi].
double Angle(const math::Quaterniond &_rot)
{
  return 2.0 * std::acos(std::min(1.0, std::abs(_rot.W())));
}

/// \brief Run a world and return the pose of each body's link at the end.
/// \param[in] _world World file, in the test worlds.
/// \param[in] _steps Steps of 1 ms to run.
/// \param[in] _bodies Model names.
/// \return Link pose by model name.
std::map<std::string, math::Pose3d> FinalPoses(
    const std::string &_world, const int _steps,
    const std::vector<std::string> &_bodies)
{
  TestFixture fixture(common::joinPaths(TEST_WORLD_DIR, _world));
  std::map<std::string, math::Pose3d> pose;
  fixture.OnPostUpdate([&](const UpdateInfo &,
                           const EntityComponentManager &_ecm)
  {
    for (const auto &name : _bodies)
      pose[name] = worldPose(LinkOf(_ecm, name).Entity(), _ecm);
  }).Finalize();
  fixture.Server()->Run(true, _steps, false);
  return pose;
}
}  // namespace

//////////////////////////////////////////////////
/// Thrust and drag on one line leave the body level at its terminal speed;
/// drag left at the link origin, above the thrust, pitches it.
TEST(HydrodynamicsCenterOfPressure, DragActsAtThePoint)
{
  TestFixture fixture(common::joinPaths(TEST_WORLD_DIR,
                                        "center_of_pressure.sdf"));
  std::map<std::string, math::Pose3d> pose;
  std::map<std::string, math::Vector3d> velocity;
  fixture.OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
  {
    for (const std::string name : {"at_point", "at_origin"})
    {
      // Without a position the force acts at the centre of mass.
      LinkOf(_ecm, name).AddWorldForce(_ecm,
                                       math::Vector3d(kThrust, 0, 0));
    }
  }).OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
  {
    for (const std::string name : {"at_point", "at_origin"})
    {
      const Link link = LinkOf(_ecm, name);
      pose[name] = worldPose(link.Entity(), _ecm);
      if (const auto v = link.WorldLinearVelocity(_ecm))
        velocity[name] = *v;
    }
  }).Finalize();
  fixture.Server()->Run(true, 5000, false);

  // Level, at the speed where the drag carries the thrust.
  EXPECT_NEAR(0.0, pose["at_point"].Rot().Pitch(), 1e-6);
  EXPECT_NEAR(std::sqrt(kThrust / kDamping), velocity["at_point"].X(), 1e-2);
  EXPECT_NEAR(0.0, velocity["at_point"].Z(), 1e-6);

  // The same body with the model at the link origin does not stay level:
  // it turns by more than a radian, whichever way the angle is read.
  EXPECT_GT(Angle(pose["at_origin"].Rot()), 1.0);
}

//////////////////////////////////////////////////
/// A body turned about its centre of mass feels no linear drag when its
/// model is written there: the point does not move. With the model left at
/// the link origin, which swings around the centre of mass, the drag there
/// pushes the body away.
TEST(HydrodynamicsCenterOfPressure, VelocityIsTakenAtThePoint)
{
  TestFixture fixture(common::joinPaths(TEST_WORLD_DIR,
                                        "center_of_pressure.sdf"));
  const math::Vector3d centreOfMass(0, 0, -0.1);
  std::map<std::string, math::Vector3d> start;
  std::map<std::string, math::Vector3d> end;
  fixture.OnPreUpdate([&](const UpdateInfo &, EntityComponentManager &_ecm)
  {
    for (const std::string name : {"at_point", "at_origin"})
    {
      // A pure pitching moment.
      LinkOf(_ecm, name).AddWorldWrench(_ecm, math::Vector3d::Zero,
                                        math::Vector3d(0, 1, 0));
    }
  }).OnPostUpdate([&](const UpdateInfo &, const EntityComponentManager &_ecm)
  {
    for (const std::string name : {"at_point", "at_origin"})
    {
      const auto pose = worldPose(LinkOf(_ecm, name).Entity(), _ecm);
      end[name] = pose.Pos() + pose.Rot() * centreOfMass;
      if (start.find(name) == start.end())
        start[name] = end[name];
    }
  }).Finalize();
  fixture.Server()->Run(true, 5000, false);

  // A fraction of a millimetre, which the integrator leaves, against
  // centimetres.
  EXPECT_LT(start["at_point"].Distance(end["at_point"]), 1e-3)
      << start["at_point"].Distance(end["at_point"]);
  EXPECT_GT(start["at_origin"].Distance(end["at_origin"]), 2e-2)
      << start["at_origin"].Distance(end["at_origin"]);
}

//////////////////////////////////////////////////
/// In a current that grows with depth, a body whose model is written below
/// its centre of mass is pushed by the water at that point; one that names
/// no point is pushed by the water at its centre of mass, not at its link
/// origin.
TEST(HydrodynamicsCenterOfPressure, CurrentIsTakenAtThePoint)
{
  TestFixture fixture(common::joinPaths(TEST_WORLD_DIR,
                                        "current_shear.sdf"));
  std::map<std::string, math::Vector3d> velocity;
  fixture.OnPostUpdate([&](const UpdateInfo &,
                           const EntityComponentManager &_ecm)
  {
    for (const std::string name : {"at_point", "at_origin", "bad_tag"})
    {
      if (const auto v = LinkOf(_ecm, name).WorldLinearVelocity(_ecm))
        velocity[name] = *v;
    }
  }).Finalize();
  // The table is loaded on the first step; the drag acts from then on.
  const int steps{100};
  fixture.Server()->Run(true, steps + 1, false);

  // From rest, linear drag of 1 kg/s on 10 kg: v = current * t / 10, as
  // long as the body is far slower than the water.
  const double elapsed = steps * 1e-3;
  EXPECT_NEAR(0.75 * elapsed / 10.0, velocity["at_point"].X(), 2e-4);
  EXPECT_NEAR(0.625 * elapsed / 10.0, velocity["at_origin"].X(), 2e-4);
  // A point that does not parse is ignored, not read as the link origin,
  // where the water runs at 0.5 m/s.
  EXPECT_NEAR(0.625 * elapsed / 10.0, velocity["bad_tag"].X(), 2e-4);
}

//////////////////////////////////////////////////
/// The tag is a change of reference point and nothing else: a body that
/// names a point moves like the same body with its link frame put at that
/// point, with added mass, Coriolis, damping and a current all at work on
/// a tilted, tumbling body. They differ by what the integrator makes of two
/// descriptions of one motion, which shrinks with the step. The body that
/// leaves the model at the link origin goes somewhere else.
TEST(HydrodynamicsCenterOfPressure, MatchesAModelWrittenAtThePoint)
{
  const auto pose = FinalPoses("center_of_pressure_equivalent.sdf", 2000,
                               {"tag", "moved", "plain"});
  // Centre of mass in each link frame, and where each model starts.
  const math::Vector3d com(0.05, 0.02, -0.10);
  const math::Vector3d point(0.10, -0.03, -0.20);
  const auto centre = [&](const std::string &_name,
                          const math::Vector3d &_com, const double _y)
  {
    const auto &p = pose.at(_name);
    return p.Pos() + p.Rot() * _com - math::Vector3d(0, _y, 0);
  };
  const auto tag = centre("tag", com, 0);
  const auto moved = centre("moved", com - point, 10);
  const auto plain = centre("plain", com, 20);

  EXPECT_GT(Angle(pose.at("tag").Rot()), 0.5);
  EXPECT_LT(tag.Distance(moved), 1e-3) << tag << " against " << moved;
  EXPECT_LT(Angle(pose.at("tag").Rot().Inverse() * pose.at("moved").Rot()),
            2e-3);
  EXPECT_GT(tag.Distance(plain), 0.05) << tag << " against " << plain;
}

//////////////////////////////////////////////////
/// The correction for the physics engine's added mass belongs to the link:
/// two bodies that differ only in their centre of pressure, with nothing
/// else in the plugin, move alike. The body without the plugin, whose
/// Coriolis terms stay against the ground, ends somewhere else.
TEST(HydrodynamicsCenterOfPressure, FluidAddedMassIgnoresThePoint)
{
  const auto pose = FinalPoses("center_of_pressure_fluid_added_mass.sdf",
                               3000, {"at_point", "at_origin", "no_plugin"});
  const auto moved = [&](const std::string &_name, const double _y)
  {
    return pose.at(_name).Pos() - math::Vector3d(0, _y, 0);
  };
  EXPECT_GT(moved("at_origin", 10).Distance(moved("no_plugin", 20)), 1e-2);
  EXPECT_LT(moved("at_point", 0).Distance(moved("at_origin", 10)), 1e-6);
  EXPECT_LT(Angle(pose.at("at_point").Rot().Inverse() *
                  pose.at("at_origin").Rot()), 1e-6);
}

//////////////////////////////////////////////////
/// The world's own current reaches a body that names a point: it settles
/// at the water's velocity, 0.5 m/s east.
TEST(HydrodynamicsCenterOfPressure, FeelsTheWorldCurrentAtThePoint)
{
  TestFixture fixture(common::joinPaths(
      TEST_WORLD_DIR, "center_of_pressure_world_current.sdf"));
  math::Vector3d velocity;
  fixture.OnPostUpdate([&](const UpdateInfo &,
                           const EntityComponentManager &_ecm)
  {
    if (const auto v = LinkOf(_ecm, "at_point").WorldLinearVelocity(_ecm))
      velocity = *v;
  }).Finalize();
  fixture.Server()->Run(true, 3000, false);

  EXPECT_NEAR(0.5, velocity.X(), 1e-2);
  EXPECT_NEAR(0.0, velocity.Y(), 1e-2);
  EXPECT_NEAR(0.0, velocity.Z(), 1e-2);
}
