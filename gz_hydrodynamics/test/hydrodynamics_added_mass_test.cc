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

// The plugin's added mass against theory and against the physics engine's
// own: the Coriolis terms of the added mass turn a body that moves
// diagonally broadside to its motion, the Munk moment.
#include <gtest/gtest.h>

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

/// \brief Run a world and return the pose of each body at the end.
/// \param[in] _world World file, in the test worlds.
/// \param[in] _steps Steps of 1 ms to run.
/// \param[in] _bodies Model names.
/// \return Pose by model name.
std::map<std::string, math::Pose3d> FinalPoses(
    const std::string &_world, const int _steps,
    const std::vector<std::string> &_bodies = {"plugin", "native"})
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
/// With more added mass in sway than in surge, a body moving along +x and
/// +y is turned clockwise, towards broadside. Kirchhoff's equations give
/// (Izz + A66) dr/dt = (A11 - A22) u v, with u and v growing from rest as
/// g m t / (m + A).
TEST(HydrodynamicsAddedMass, CoriolisTurnsTheBodyBroadside)
{
  const double m{10}, a11{1}, a22{6}, a66{0.02}, izz{0.25}, gx{1}, gy{0.5};
  const double t{0.2};
  // The yaw rate integrates u v, which grows as t^2; the yaw integrates
  // that once more.
  const double uvOverT2 = gx * gy * m * m / ((m + a11) * (m + a22));
  const double yaw = (a11 - a22) * uvOverT2 * std::pow(t, 4) / 12 /
      (izz + a66);

  const auto pose = FinalPoses("added_mass.sdf", static_cast<int>(t / 1e-3));
  EXPECT_LT(pose.at("plugin").Rot().Yaw(), 0.0);
  EXPECT_NEAR(yaw, pose.at("plugin").Rot().Yaw(), 0.05 * std::abs(yaw));
}

//////////////////////////////////////////////////
/// In still water the plugin's added mass and the physics engine's are the
/// same model, so the two bodies move alike.
TEST(HydrodynamicsAddedMass, MatchesThePhysicsEngineInStillWater)
{
  const auto pose = FinalPoses("added_mass.sdf", 2000);
  const math::Vector3d offset(0, 5, 0);
  // Both moved and turned well away from where they started.
  EXPECT_GT(pose.at("native").Pos().Distance(offset), 1.0);
  EXPECT_GT(std::abs(pose.at("native").Rot().Yaw()), 0.2);
  EXPECT_LT(pose.at("plugin").Pos().Distance(
      pose.at("native").Pos() - offset), 5e-3);
  EXPECT_LT((pose.at("plugin").Rot().Inverse() *
             pose.at("native").Rot()).Euler().Length(), 5e-3);
}

//////////////////////////////////////////////////
/// A link with <fluid_added_mass> gets its Coriolis terms from the physics
/// engine. When the plugin is also given the added mass parameters, with
/// its added mass force disabled, it must not apply those terms a second
/// time: the body moves like the one with <fluid_added_mass> alone.
TEST(HydrodynamicsAddedMass, FluidAddedMassCoriolisNotCountedTwice)
{
  const auto pose = FinalPoses("added_mass.sdf", 2000, {"native", "both"});
  // "both" starts 5 m to the left of "native".
  const math::Vector3d offset(0, 5, 0);
  EXPECT_GT(std::abs(pose.at("native").Rot().Yaw()), 0.2);
  EXPECT_LT(pose.at("native").Pos().Distance(
      pose.at("both").Pos() - offset), 1e-6);
  EXPECT_LT((pose.at("native").Rot().Inverse() *
             pose.at("both").Rot()).Euler().Length(), 1e-6);
}

//////////////////////////////////////////////////
/// In a current the physics engine's added mass takes its Coriolis terms
/// against the ground. The plugin corrects them, so the body ends within
/// centimetres of the one whose added mass is in the plugin; without the
/// correction they end a quarter of a metre apart. What is left is the
/// current's turning in the body frame, which the correction leaves out.
TEST(HydrodynamicsAddedMass, CorrectsThePhysicsEngineInACurrent)
{
  const auto pose = FinalPoses("added_mass_current.sdf", 4000);
  const math::Vector3d offset(0, 5, 0);
  EXPECT_GT(pose.at("native").Pos().Distance(offset), 1.0);
  EXPECT_LT(pose.at("plugin").Pos().Distance(
      pose.at("native").Pos() - offset), 0.03);
}
