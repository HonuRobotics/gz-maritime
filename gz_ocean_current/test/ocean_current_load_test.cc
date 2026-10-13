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
// The load on collisions marked gz:ocean_current="true", in a running world:
// the water drags the part of each marked shape below the water level,
// relative to it, and nothing else.
#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/param.pb.h>

#include <chrono>
#include <map>
#include <string>
#include <thread>

#include <gz/common/Filesystem.hh>
#include <gz/math/Helpers.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/components/OceanCurrentfield.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief The current, m/s, towards +x.
constexpr double kU{0.5};

/// \brief k for a fully wetted 1 m face of a 10 kg box with a Cd of 1:
/// 0.5 * 1025 * 1 * 1 / 10.
constexpr double kK{51.25};

/// \brief The closed form of a body starting at rest under quadratic drag
/// alone, catching up with a current.
/// \param[in] _k 0.5 * rho * Cd * A / m, 1/m.
/// \param[in] _t Time, s.
/// \return Its speed, m/s.
double CatchUp(double _k, double _t)
{
  return kU - 1.0 / (1.0 / kU + _k * _t);
}

/// \brief Load the test world and keep the velocity of every model's link.
class LoadWorld
{
  /// \brief Constructor.
  public: LoadWorld()
    : fixture(common::joinPaths(TEST_WORLD_DIR, "ocean_current_marked.sdf"))
  {
    this->fixture.OnPreUpdate([](const UpdateInfo &,
        EntityComponentManager &_ecm)
    {
      // Track every link, the unmarked ones and those spawned later too.
      _ecm.Each<components::Link>(
          [&](const Entity &_link, const components::Link *) -> bool
          {
            Link(_link).EnableVelocityChecks(_ecm);
            return true;
          });
    });
    this->fixture.OnPostUpdate([this](const UpdateInfo &,
        const EntityComponentManager &_ecm)
    {
      const Entity worldEntity = _ecm.EntityByComponents(components::World());
      if (const auto *field =
          _ecm.Component<components::OceanCurrentfield>(worldEntity))
      {
        this->waterLevel = field->Data().params.water_level;
      }
      _ecm.Each<components::Model, components::Name>(
          [&](const Entity &_model, const components::Model *,
              const components::Name *_name) -> bool
          {
            const Entity link = _ecm.EntityByComponents(
                components::Link(), components::ParentEntity(_model));
            const auto *vel =
                _ecm.Component<components::WorldLinearVelocity>(link);
            if (nullptr != vel)
              this->velocity[_name->Data()] = vel->Data();
            return true;
          });
    });
    this->fixture.Finalize();
  }

  /// \brief Run some steps.
  /// \param[in] _steps Steps, of 1 ms.
  /// \return True on success.
  public: bool Run(std::size_t _steps)
  {
    return this->fixture.Server()->Run(true, _steps, false);
  }

  /// \brief The fixture.
  public: TestFixture fixture;

  /// \brief World linear velocity of each model's link, by model name.
  public: std::map<std::string, math::Vector3d> velocity;

  /// \brief The recipe's water level.
  public: double waterLevel{0.0};
};

/// \brief Ask the ocean current system to change a numeric parameter.
/// \param[in] _key Parameter name.
/// \param[in] _value Its new value.
/// \return True once the message was sent to a subscriber.
bool SetCurrent(const std::string &_key, double _value)
{
  static transport::Node node;
  auto pub = node.Advertise<msgs::Param>(
      "/world/ocean_current_marked/ocean_current/set");
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (!pub.HasConnections())
    return false;
  msgs::Param msg;
  auto &any = (*msg.mutable_params())[_key];
  any.set_type(msgs::Any::DOUBLE);
  any.set_double_value(_value);
  return pub.Publish(msg);
}

/// \brief A marked box like the world's, to spawn at run time.
const char kLateBox[] = R"(<?xml version="1.0"?>
  <sdf version="1.9" xmlns:gz="http://gazebosim.org/schema">
    <model name="late">
      <pose>0 -3 -2 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1000</ixx><iyy>1000</iyy><izz>1000</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="hull" gz:ocean_current="true">
          <geometry><box><size>1 1 1</size></box></geometry>
          <surface>
            <contact><collide_bitmask>0x00</collide_bitmask></contact>
          </surface>
        </collision>
      </link>
    </model>
  </sdf>)";

/// \brief Tolerance against the closed form, m/s: the integrator lags it by
/// a step or so.
constexpr double kTol{0.01};
}  // namespace

/////////////////////////////////////////////////
/// A submerged marked box catches up with the water as quadratic drag on
/// its whole face says, along the current only.
TEST(OceanCurrentLoad, SubmergedShapeIsDraggedByTheWater)
{
  LoadWorld world;
  ASSERT_TRUE(world.Run(200));
  const math::Vector3d v = world.velocity["submerged"];
  EXPECT_NEAR(CatchUp(kK, 0.2), v.X(), kTol);
  EXPECT_NEAR(0.0, v.Y(), 1e-6);
  EXPECT_NEAR(0.0, v.Z(), 1e-6);

  // And settles at the current's speed.
  ASSERT_TRUE(world.Run(2800));
  EXPECT_NEAR(kU, world.velocity["submerged"].X(), kTol);
}

/////////////////////////////////////////////////
/// A box half under the water takes half the area the current sees.
TEST(OceanCurrentLoad, OnlyThePartBelowTheWater)
{
  LoadWorld world;
  ASSERT_TRUE(world.Run(200));
  EXPECT_NEAR(CatchUp(kK / 2.0, 0.2), world.velocity["half"].X(), kTol);
}

/////////////////////////////////////////////////
/// gz:ocean_current_cd sets the shape's drag coefficient.
TEST(OceanCurrentLoad, DragCoefficientAttribute)
{
  LoadWorld world;
  ASSERT_TRUE(world.Run(200));
  EXPECT_NEAR(CatchUp(2.0 * kK, 0.2), world.velocity["draggy"].X(), kTol);
}

/////////////////////////////////////////////////
/// A marked box clear of the water and an unmarked box in it are left
/// alone.
TEST(OceanCurrentLoad, OnlyMarkedShapesInTheWater)
{
  LoadWorld world;
  ASSERT_TRUE(world.Run(500));
  EXPECT_NEAR(0.0, world.velocity["above"].Length(), 1e-9);
  EXPECT_NEAR(0.0, world.velocity["unmarked"].Length(), 1e-9);
}

/////////////////////////////////////////////////
/// A marked box spawned after the world has run a while is found and
/// dragged like one loaded with the world.
TEST(OceanCurrentLoad, ShapeSpawnedLaterIsFound)
{
  LoadWorld world;
  ASSERT_TRUE(world.Run(100));

  transport::Node node;
  msgs::EntityFactory req;
  req.set_sdf(kLateBox);
  msgs::Boolean rep;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/ocean_current_marked/create", req, 5000,
                           rep, result));
  ASSERT_TRUE(result && rep.data());

  ASSERT_TRUE(world.Run(3000));
  ASSERT_EQ(1u, world.velocity.count("late"));
  EXPECT_NEAR(kU, world.velocity["late"].X(), kTol);
  EXPECT_NEAR(0.0, world.velocity["late"].Y(), 1e-6);
}

/////////////////////////////////////////////////
/// A shape on its side is cut on the face the current sees: a box pitched
/// onto its side and a cylinder lying along the current, each half under
/// the water, take half their face, as the upright box does.
TEST(OceanCurrentLoad, ShapeOnItsSide)
{
  LoadWorld world;
  ASSERT_TRUE(world.Run(200));
  EXPECT_NEAR(CatchUp(kK / 2.0, 0.2), world.velocity["pitched"].X(), kTol);
  // The end cap of a 0.5 m radius cylinder, half of it wet: pi / 8 m^2.
  EXPECT_NEAR(CatchUp(kK * GZ_PI / 8.0, 0.2), world.velocity["lying"].X(),
              kTol);
}

/////////////////////////////////////////////////
/// Changing <water_level> on the ocean current topic moves the cut: raised
/// half a metre, the box half under the water is all under it, and keeps up
/// with the one that always was.
TEST(OceanCurrentLoad, WaterLevelOnTheTopicMovesTheCut)
{
  LoadWorld world;
  ASSERT_TRUE(world.Run(1));
  ASSERT_TRUE(SetCurrent("water_level", 0.5));
  for (int i = 0; i < 2000 && world.waterLevel != 0.5; ++i)
    ASSERT_TRUE(world.Run(1));
  ASSERT_EQ(0.5, world.waterLevel) << "the change never arrived";

  ASSERT_TRUE(world.Run(200));
  EXPECT_NEAR(world.velocity["submerged"].X(), world.velocity["half"].X(),
              kTol);
  EXPECT_GT(world.velocity["half"].X(), CatchUp(kK / 2.0, 0.2) + 2 * kTol)
      << "faster than the half wetted box it was";
}

/////////////////////////////////////////////////
/// After a reset the marked shapes are found again and dragged again from
/// rest.
TEST(OceanCurrentLoad, MarkedSurvivesReset)
{
  LoadWorld world;
  ASSERT_TRUE(world.Run(500));
  ASSERT_GT(world.velocity["submerged"].X(), CatchUp(kK, 0.2));

  world.fixture.Server()->ResetAll();
  ASSERT_TRUE(world.Run(200));
  EXPECT_NEAR(CatchUp(kK, 0.2), world.velocity["submerged"].X(), kTol);
  EXPECT_NEAR(CatchUp(kK / 2.0, 0.2), world.velocity["half"].X(), kTol);
  EXPECT_NEAR(0.0, world.velocity["unmarked"].Length(), 1e-9);
}
