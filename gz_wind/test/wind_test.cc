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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/wind.pb.h>

#include <chrono>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <gz/common/Filesystem.hh>
#include <gz/math/Vector3.hh>

#include <gz/transport/Node.hh>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief What one iteration of the world tells us about a box.
struct BoxState
{
  /// \brief World position of the model origin.
  public: math::Vector3d pos;

  /// \brief World linear velocity of its link, zero when not tracked.
  public: math::Vector3d vel;

  /// \brief World angular velocity of its link, zero when not tracked.
  public: math::Vector3d angVel;

  /// \brief Whether the link's velocity is tracked, which the wind system
  /// turns on for every link it pushes.
  public: bool tracked{false};

  /// \brief Whether the model was found at all.
  public: bool found{false};
};

/// \brief Read a model's state out of the ECM.
/// \param[in] _ecm Entity component manager.
/// \param[in] _model Name of the model to look up.
/// \return Its state this iteration.
BoxState ReadBox(const EntityComponentManager &_ecm, const std::string &_model)
{
  BoxState state;

  const Entity model = _ecm.EntityByComponents(
      components::Model(), components::Name(_model));
  if (kNullEntity == model)
    return state;

  const Entity link = _ecm.EntityByComponents(
      components::Link(), components::Name("link"),
      components::ParentEntity(model));
  if (kNullEntity == link)
    return state;

  state.found = true;
  state.pos = worldPose(model, _ecm).Pos();
  const auto *vel = _ecm.Component<components::WorldLinearVelocity>(link);
  state.tracked = nullptr != vel;
  if (state.tracked)
    state.vel = vel->Data();
  if (const auto *ang = _ecm.Component<components::WorldAngularVelocity>(link))
    state.angVel = ang->Data();

  return state;
}

/// \brief The wind the wind entity holds this iteration.
/// \param[in] _ecm Entity component manager.
/// \return Its linear velocity, zero if there is none.
math::Vector3d ReadWind(const EntityComponentManager &_ecm)
{
  const Entity wind = _ecm.EntityByComponents(components::Wind());
  const auto *vel = _ecm.Component<components::WorldLinearVelocity>(wind);
  return nullptr == vel ? math::Vector3d::Zero : vel->Data();
}

/// \brief Ask the wind system to change the wind.
/// \param[in] _world World name.
/// \param[in] _key Parameter, speed or direction.
/// \param[in] _value Its new value.
/// \return The service's answer.
bool SetWind(const std::string &_world, const std::string &_key,
             double _value)
{
  msgs::Param req;
  auto &any = (*req.mutable_params())[_key];
  any.set_type(msgs::Any::DOUBLE);
  any.set_double_value(_value);
  transport::Node node;
  msgs::Boolean rep;
  bool result{false};
  return node.Request("/world/" + _world + "/wind/set_parameters", req,
      5000, rep, result) && result && rep.data();
}

/// \brief Path to one of the test worlds.
/// \param[in] _file World file name.
/// \return Its absolute path.
std::string World(const std::string &_file)
{
  return common::joinPaths(TEST_WORLD_DIR, _file);
}

/// \brief Speed a 100 kg cube of 1 m reaches after one second in a 5 m/s
/// wind, from 0.5 * 1.225 * 1 * 1 * 25 N, before the relative wind shrinks
/// as the cube picks up speed.
constexpr double kSpeedAfterOneSecond{0.153};

/// \brief A 1 m, 100 kg box with a contact collision and a marked one, as a
/// vehicle would carry.
const char kMarkedBox[] = R"(<?xml version="1.0"?>
  <sdf version="1.9" xmlns:gz="http://gazebosim.org/schema">
    <model name="marked_box">
      <pose>0 0 0.5 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>100</mass>
          <inertia>
            <ixx>16.667</ixx><iyy>16.667</iyy><izz>16.667</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>
        <collision name="contact">
          <geometry><box><size>1 1 1</size></box></geometry>
        </collision>
        <collision name="windage" gz:wind="true">
          <geometry><box><size>1 1 1</size></box></geometry>
          <surface><contact><collide_bitmask>0x00</collide_bitmask></contact></surface>
        </collision>
      </link>
    </model>
  </sdf>)";
}  // namespace

/////////////////////////////////////////////////
/// The wind moves only the boxes that mark a collision, along the wind, by
/// the drag on the marked area. Half of a face under the waterline takes
/// half the force.
TEST(Windage, MarkedCollisions)
{
  TestFixture fixture(World("windage.sdf"));

  BoxState plain;
  BoxState marked;
  BoxState half;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    plain = ReadBox(_ecm, "plain_box");
    marked = ReadBox(_ecm, "marked_box");
    half = ReadBox(_ecm, "half_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 500, false));

  ASSERT_TRUE(plain.found);
  EXPECT_FALSE(plain.tracked) << "a box that marks nothing is left alone";
  EXPECT_NEAR(0.0, plain.pos.X(), 1e-6);

  ASSERT_TRUE(marked.found);
  EXPECT_TRUE(marked.tracked);
  EXPECT_NEAR(kSpeedAfterOneSecond, marked.vel.X(), 0.01);
  EXPECT_NEAR(0.0, marked.vel.Y(), 1e-6) << "nothing pushes across the wind";
  EXPECT_NEAR(0.0, marked.vel.Z(), 1e-6) << "nor up";
  EXPECT_GT(marked.pos.X(), 0.05);

  ASSERT_TRUE(half.found);
  EXPECT_TRUE(half.tracked);
  EXPECT_NEAR(kSpeedAfterOneSecond / 2.0, half.vel.X(), 0.005)
      << "only the face above the waterline is in the wind";
}

/////////////////////////////////////////////////
/// A shape sets its own drag coefficient with gz:wind_cd.
TEST(Windage, DragCoefficientAttribute)
{
  TestFixture fixture(World("windage.sdf"));

  BoxState marked;
  BoxState cd;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    marked = ReadBox(_ecm, "marked_box");
    cd = ReadBox(_ecm, "cd_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 500, false));

  ASSERT_TRUE(marked.found);
  ASSERT_TRUE(cd.found);
  EXPECT_TRUE(cd.tracked);
  EXPECT_NEAR(2.0 * kSpeedAfterOneSecond, cd.vel.X(), 0.02);
  EXPECT_GT(cd.vel.X(), 1.8 * marked.vel.X());
}

/////////////////////////////////////////////////
/// A vehicle spawned into a running world, under a name that is not the one
/// in its file, is pushed like one loaded with the world: the mark travels
/// with the model.
TEST(Windage, MarkedSpawnedByAnyName)
{
  TestFixture fixture(World("windage.sdf"));

  BoxState spawned;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    spawned = ReadBox(_ecm, "usv_a");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 10, false));
  ASSERT_FALSE(spawned.found);

  msgs::EntityFactory req;
  req.set_sdf(kMarkedBox);
  req.set_name("usv_a");
  req.set_allow_renaming(false);
  req.mutable_pose()->mutable_position()->set_y(20.0);
  req.mutable_pose()->mutable_position()->set_z(0.5);

  transport::Node node;
  msgs::Boolean rep;
  bool result{false};
  ASSERT_TRUE(node.Request("/world/windage/create", req, 5000, rep, result));
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());

  ASSERT_TRUE(server->Run(true, 500, false));

  ASSERT_TRUE(spawned.found) << "the model should have been created";
  EXPECT_TRUE(spawned.tracked);
  EXPECT_NEAR(kSpeedAfterOneSecond, spawned.vel.X(), 0.01);
  EXPECT_NEAR(20.0, spawned.pos.Y(), 1e-6);
}

/////////////////////////////////////////////////
/// After a reset the marked boxes are found again and pushed again.
TEST(Windage, MarkedSurvivesReset)
{
  TestFixture fixture(World("windage.sdf"));

  BoxState plain;
  BoxState marked;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    plain = ReadBox(_ecm, "plain_box");
    marked = ReadBox(_ecm, "marked_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 100, false));
  ASSERT_TRUE(marked.tracked);
  EXPECT_GT(marked.vel.X(), 0.0);

  server->ResetAll();
  ASSERT_TRUE(server->Run(true, 500, false));

  EXPECT_FALSE(plain.tracked) << "still left alone after the reset";
  EXPECT_NEAR(0.0, plain.pos.X(), 1e-6);
  EXPECT_TRUE(marked.tracked) << "pushed again after the reset";
  EXPECT_NEAR(kSpeedAfterOneSecond, marked.vel.X(), 0.01);
}

/////////////////////////////////////////////////
/// The force acts at the shape, not at the link: a marked shape above the
/// centre of mass turns its link about the axis across the wind, and one
/// centred on the centre of mass does not, even when the centre of mass is
/// off the link origin.
TEST(Windage, MomentFromTheShapePosition)
{
  TestFixture fixture(World("windage.sdf"));

  BoxState tall;
  BoxState com;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    tall = ReadBox(_ecm, "tall_box");
    com = ReadBox(_ecm, "com_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_NE(nullptr, server);
  ASSERT_TRUE(server->Run(true, 500, false));

  ASSERT_TRUE(tall.found);
  ASSERT_TRUE(tall.tracked);
  // 15.3 N at 2 m on a 16.667 kg m^2 axis starts it at 1.8 rad/s^2; as it
  // turns, the shape moves downwind at 2 m times the rate, which cuts the
  // relative wind and the moment, so it settles near 1 rad/s after 1 s.
  EXPECT_GT(tall.angVel.Y(), 0.5)
      << "a wind along +x heels a tall shape about +y";
  EXPECT_LT(tall.angVel.Y(), 1.84) << "never faster than the starting moment";
  EXPECT_NEAR(0.0, tall.angVel.X(), 1e-6);
  EXPECT_NEAR(0.0, tall.angVel.Z(), 1e-6);

  ASSERT_TRUE(com.found);
  ASSERT_TRUE(com.tracked);
  EXPECT_GT(com.vel.X(), 0.1) << "it is pushed";
  EXPECT_NEAR(0.0, com.angVel.Y(), 1e-3) << "but not turned";
}

/////////////////////////////////////////////////
/// A world that only sets <wind> keeps it: the wind system starts from it.
TEST(WindField, WorldWindKeptWithoutParameters)
{
  TestFixture fixture(World("windage.sdf"));

  math::Vector3d wind;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    wind = ReadWind(_ecm);
  });
  fixture.Finalize();

  ASSERT_TRUE(fixture.Server()->Run(true, 10, false));
  EXPECT_NEAR(5.0, wind.X(), 1e-9);
  EXPECT_NEAR(0.0, wind.Y(), 1e-9);
}

/////////////////////////////////////////////////
/// <speed> and <direction> set the wind: 270, a wind from the west, blows
/// towards +x, and pushes the marked box that way.
TEST(WindField, SpeedAndDirectionFromTheWorld)
{
  TestFixture fixture(World("windfield.sdf"));

  math::Vector3d wind;
  BoxState box;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    wind = ReadWind(_ecm);
    box = ReadBox(_ecm, "marked_box");
  });
  fixture.Finalize();

  ASSERT_TRUE(fixture.Server()->Run(true, 500, false));
  EXPECT_NEAR(5.0, wind.X(), 1e-9);
  EXPECT_NEAR(0.0, wind.Y(), 1e-9);
  ASSERT_TRUE(box.found);
  EXPECT_NEAR(kSpeedAfterOneSecond, box.vel.X(), 0.01);
  EXPECT_NEAR(0.0, box.vel.Y(), 1e-6);
}

/////////////////////////////////////////////////
/// The service changes the wind while the world runs: a wind from the south
/// pushes the box north, and after a zero speed the still air only brakes it.
TEST(WindField, SetParametersAtRunTime)
{
  TestFixture fixture(World("windfield.sdf"));

  math::Vector3d wind;
  BoxState box;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    wind = ReadWind(_ecm);
    box = ReadBox(_ecm, "marked_box");
  });
  fixture.Finalize();

  auto server = fixture.Server();
  ASSERT_TRUE(server->Run(true, 1, false));
  ASSERT_TRUE(SetWind("windfield", "direction", 180.0));
  ASSERT_TRUE(server->Run(true, 500, false));
  EXPECT_NEAR(0.0, wind.X(), 1e-9);
  EXPECT_NEAR(5.0, wind.Y(), 1e-9);
  EXPECT_GT(box.vel.Y(), 0.1) << "a wind from the south pushes north";

  ASSERT_TRUE(SetWind("windfield", "speed", 0.0));
  ASSERT_TRUE(server->Run(true, 2, false));
  const math::Vector3d still = box.vel;
  ASSERT_TRUE(server->Run(true, 200, false));
  EXPECT_NEAR(0.0, wind.Length(), 1e-9);
  EXPECT_LT(box.vel.Y(), still.Y()) << "still air brakes a moving box";
  EXPECT_GT(box.vel.Y(), 0.0) << "but never pushes it back";

  EXPECT_FALSE(SetWind("windfield", "gust", 3.0)) << "unknown keys are refused";
}

/////////////////////////////////////////////////
/// The wind is published as ground truth on /world/<world>/wind_info.
TEST(WindField, GroundTruthIsPublished)
{
  std::mutex mutex;
  std::optional<msgs::Wind> received;
  transport::Node node;
  ASSERT_TRUE(node.Subscribe("/world/windfield/wind_info",
      std::function<void(const msgs::Wind &)>(
      [&](const msgs::Wind &_msg)
      {
        const std::lock_guard<std::mutex> lock(mutex);
        received = _msg;
      })));

  TestFixture fixture(World("windfield.sdf"));
  fixture.Finalize();
  ASSERT_TRUE(fixture.Server()->Run(true, 200, false));

  for (int i = 0; i < 50; ++i)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      if (received)
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  const std::lock_guard<std::mutex> lock(mutex);
  ASSERT_TRUE(received.has_value());
  EXPECT_NEAR(5.0, received->linear_velocity().x(), 1e-9);
  EXPECT_NEAR(0.0, received->linear_velocity().y(), 1e-9);
}
