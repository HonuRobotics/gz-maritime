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
#include <gz/msgs/world_control.pb.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/math/CoordinateVector3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/Util.hh>

#include "gz/sim/components/Windfield.hh"
#include "gz/sim/wind/WindSampler.hh"

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

/// \brief What one iteration of the world tells us about its wind.
struct WindState
{
  /// \brief The wind entity's velocity, world frame.
  public: math::Vector3d entity;

  /// \brief The wind asked through the recipe 10 m above the world's
  /// origin, where the wind entity's wind is taken.
  public: math::Vector3d sampled;

  /// \brief The wind asked through the recipe 1 m above the world's origin.
  public: math::Vector3d low;

  /// \brief The recipe, if the world has one.
  public: std::optional<wind::WindfieldData> recipe;

  /// \brief The world's spherical coordinates, if it has them.
  public: std::optional<math::SphericalCoordinates> sc;
};

/// \brief Read the wind out of the ECM.
/// \param[in] _info Update info.
/// \param[in] _ecm Entity component manager.
/// \return The wind this iteration.
WindState ReadWind(const UpdateInfo &_info, const EntityComponentManager &_ecm)
{
  WindState state;
  const Entity windEntity = _ecm.EntityByComponents(components::Wind());
  if (const auto *vel =
      _ecm.Component<components::WorldLinearVelocity>(windEntity))
  {
    state.entity = vel->Data();
  }
  state.sampled = wind::WindAt(_ecm, {0, 0, 10}, _info.simTime);
  state.low = wind::WindAt(_ecm, {0, 0, 1}, _info.simTime);
  const Entity world = _ecm.EntityByComponents(components::World());
  if (const auto *field = _ecm.Component<components::Windfield>(world))
    state.recipe = field->Data();
  if (const auto *sc = _ecm.Component<components::SphericalCoordinates>(world))
    state.sc = sc->Data();
  return state;
}

/// \brief Ask the wind system to change the wind, on its topic.
/// \param[in] _world World name.
/// \param[in] _key Parameter name.
/// \param[in] _value Its new value.
/// \return True once the message was sent to a subscriber.
bool SetWind(const std::string &_world, const std::string &_key,
             double _value)
{
  static transport::Node node;
  auto pub = node.Advertise<msgs::Param>("/world/" + _world + "/wind/set");
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (!pub.HasConnections())
    return false;
  msgs::Param msg;
  auto &any = (*msg.mutable_params())[_key];
  any.set_type(msgs::Any::DOUBLE);
  any.set_double_value(_value);
  const bool sent = pub.Publish(msg);
  // Delivery is asynchronous; give the subscriber a moment to queue it.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return sent;
}

/// \brief Ask a world to reset, as the GUI's reset button does.
/// \param[in] _world World name.
/// \return True if the world took the request.
bool ResetWorld(const std::string &_world)
{
  transport::Node node;
  msgs::WorldControl req;
  req.mutable_reset()->set_all(true);
  msgs::Boolean rep;
  bool result{false};
  return node.Request("/world/" + _world + "/control", req, 2000, rep,
                      result) && result && rep.data();
}

/// \brief Path to one of the test worlds.
/// \param[in] _file World file name.
/// \return Its absolute path.
std::string World(const std::string &_file)
{
  return common::joinPaths(TEST_WORLD_DIR, _file);
}

/// \brief Load a world and keep its wind up to date.
class WindWorld
{
  /// \brief Constructor.
  /// \param[in] _file World file name.
  public: explicit WindWorld(const std::string &_file)
    : fixture(World(_file))
  {
    this->fixture.OnPostUpdate([this](const UpdateInfo &_info,
        const EntityComponentManager &_ecm)
    {
      this->state = ReadWind(_info, _ecm);
      this->series.push_back(this->state.entity);
    });
    this->fixture.Finalize();
  }

  /// \brief Run some steps.
  /// \param[in] _steps Steps.
  /// \return True on success.
  public: bool Run(std::size_t _steps)
  {
    return this->fixture.Server()->Run(true, _steps, false);
  }

  /// \brief The fixture.
  public: TestFixture fixture;

  /// \brief The wind entity's velocity after every step.
  public: std::vector<math::Vector3d> series;

  /// \brief The wind after the last step.
  public: WindState state;
};

/// \brief Standard deviation of the horizontal speed of a series of winds.
/// \param[in] _v The winds.
/// \return The standard deviation, m/s.
double SpeedSd(const std::vector<math::Vector3d> &_v)
{
  double mean{0.0};
  for (const auto &w : _v)
    mean += std::hypot(w.X(), w.Y());
  mean /= static_cast<double>(_v.size());
  double var{0.0};
  for (const auto &w : _v)
  {
    const double d = std::hypot(w.X(), w.Y()) - mean;
    var += d * d;
  }
  return std::sqrt(var / static_cast<double>(_v.size()));
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
  BoxState one;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    plain = ReadBox(_ecm, "plain_box");
    marked = ReadBox(_ecm, "marked_box");
    half = ReadBox(_ecm, "half_box");
    one = ReadBox(_ecm, "one_box");
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

  ASSERT_TRUE(one.found);
  EXPECT_TRUE(one.tracked) << "a mark is a bool, so \"1\" marks too";
  EXPECT_NEAR(marked.vel.X(), one.vel.X(), 1e-9);
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
  WindWorld world("world_wind.sdf");
  ASSERT_TRUE(world.Run(10));
  EXPECT_NEAR(5.0, world.state.entity.X(), 1e-9);
  EXPECT_NEAR(0.0, world.state.entity.Y(), 1e-9);
  ASSERT_TRUE(world.state.recipe.has_value());
  EXPECT_NEAR(5.0, world.state.recipe->params.speed, 1e-9);
  EXPECT_NEAR(270.0, world.state.recipe->params.direction, 1e-9);
}

/////////////////////////////////////////////////
/// <speed> and <direction> set the wind: 270, a wind from the west, blows
/// towards +x. The recipe sits on the world, and the wind asked through it
/// is the wind the entity holds.
TEST(WindField, SpeedAndDirectionFromTheWorld)
{
  WindWorld world("windfield.sdf");
  ASSERT_TRUE(world.Run(10));
  EXPECT_NEAR(5.0, world.state.entity.X(), 1e-9);
  EXPECT_NEAR(0.0, world.state.entity.Y(), 1e-9);
  ASSERT_TRUE(world.state.recipe.has_value());
  EXPECT_EQ("standard", world.state.recipe->model);
  EXPECT_NE(0u, world.state.recipe->params.seed)
      << "a 0 seed is resolved before the recipe is written";
  EXPECT_EQ(world.state.entity, world.state.sampled);
}

/////////////////////////////////////////////////
/// The topic changes the wind while the world runs, as a new recipe; an
/// unknown key or an out of range value changes nothing.
TEST(WindField, ChangedAtRunTimeOnItsTopic)
{
  WindWorld world("windfield.sdf");
  ASSERT_TRUE(world.Run(1));
  ASSERT_TRUE(world.state.recipe.has_value());
  const auto generation = world.state.recipe->generation;

  ASSERT_TRUE(SetWind("windfield", "direction", 180.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(0.0, world.state.entity.X(), 1e-9);
  EXPECT_NEAR(5.0, world.state.entity.Y(), 1e-9)
      << "a wind from the south blows north";
  EXPECT_GT(world.state.recipe->generation, generation);
  EXPECT_EQ(world.state.entity, world.state.sampled);

  ASSERT_TRUE(SetWind("windfield", "speed", 0.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(0.0, world.state.entity.Length(), 1e-9);

  const auto before = world.state.recipe->generation;
  ASSERT_TRUE(SetWind("windfield", "gust", 3.0));
  ASSERT_TRUE(SetWind("windfield", "speed", -1.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_EQ(before, world.state.recipe->generation);
  EXPECT_NEAR(0.0, world.state.entity.Length(), 1e-9);
}

/////////////////////////////////////////////////
/// A reset brings back the world file's wind, under a new generation, and a
/// change after it starts from that wind, not from the one before.
TEST(WindField, ResetRestoresTheWorldsWind)
{
  WindWorld world("windfield.sdf");
  ASSERT_TRUE(world.Run(1));
  ASSERT_TRUE(SetWind("windfield", "speed", 8.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(8.0, world.state.entity.X(), 1e-9);
  ASSERT_TRUE(world.state.recipe.has_value());
  const auto generation = world.state.recipe->generation;

  ASSERT_TRUE(ResetWorld("windfield"));
  ASSERT_TRUE(world.Run(3));
  EXPECT_NEAR(5.0, world.state.entity.X(), 1e-9);
  EXPECT_NEAR(5.0, world.state.recipe->params.speed, 1e-9);
  EXPECT_GT(world.state.recipe->generation, generation)
      << "the restored recipe is new to every consumer";

  ASSERT_TRUE(SetWind("windfield", "direction", 180.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(0.0, world.state.entity.X(), 1e-9);
  EXPECT_NEAR(5.0, world.state.entity.Y(), 1e-9)
      << "the speed is the world file's, not the one set before the reset";
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

  WindWorld world("windfield.sdf");
  ASSERT_TRUE(world.Run(200));

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

/////////////////////////////////////////////////
/// North is the world's north, the one its spherical coordinates and GPS
/// use, not its +y axis: in a world turned 90 degrees, a wind from the west
/// blows geographic east.
TEST(WindDirection, NorthIsTheWorldsNorth)
{
  WindWorld world("heading.sdf");
  ASSERT_TRUE(world.Run(10));

  ASSERT_TRUE(world.state.sc.has_value());
  const auto expected = world.state.sc->LocalFromGlobalVelocity(
      math::CoordinateVector3::Metric(5.0, 0.0, 0.0));
  ASSERT_TRUE(expected.has_value());
  const math::Vector3d &v = world.state.entity;
  EXPECT_NEAR(expected->X().value(), v.X(), 1e-9);
  EXPECT_NEAR(expected->Y().value(), v.Y(), 1e-9);
  EXPECT_GT(std::abs(v.Y()), 4.9) << "the heading turned it off +x";
}

/////////////////////////////////////////////////
/// A world's own <wind>, read back as a speed and a direction from north and
/// written again, is the same world frame vector, heading or not.
TEST(WindDirection, WorldWindSurvivesTheRoundTrip)
{
  WindWorld world("heading_wind.sdf");
  ASSERT_TRUE(world.Run(1));
  // The same speed again forces a new recipe from the stored speed and
  // direction.
  ASSERT_TRUE(SetWind("heading_wind", "speed", 5.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(3.0, world.state.entity.X(), 1e-9);
  EXPECT_NEAR(4.0, world.state.entity.Y(), 1e-9);
}

/////////////////////////////////////////////////
/// Gusts move the wind entity every step, and the wind asked through the
/// recipe agrees with it: every system sees the same gust.
TEST(WindGusts, EveryoneSeesTheSameGust)
{
  WindWorld world("gusts.sdf");
  ASSERT_TRUE(world.Run(1000));
  EXPECT_GT(SpeedSd(world.series), 0.3);
  EXPECT_NEAR(0.0, (world.state.entity - world.state.sampled).Length(),
              1e-9);
}

/////////////////////////////////////////////////
/// The same seed gives the same gusts, run after run.
TEST(WindGusts, SeedRepeatsTheSeries)
{
  WindWorld first("gusts.sdf");
  ASSERT_TRUE(first.Run(1000));
  WindWorld second("gusts.sdf");
  ASSERT_TRUE(second.Run(1000));
  ASSERT_EQ(first.series.size(), second.series.size());
  for (std::size_t i = 0; i < first.series.size(); ++i)
    ASSERT_EQ(first.series[i], second.series[i]) << "step " << i;
}

/////////////////////////////////////////////////
/// The gusts are a function of time, so a reset replays them.
TEST(WindGusts, ResetReplaysTheGusts)
{
  WindWorld world("gusts.sdf");
  ASSERT_TRUE(world.Run(500));
  const std::vector<math::Vector3d> before = world.series;
  world.series.clear();
  world.fixture.Server()->ResetAll();
  ASSERT_TRUE(world.Run(500));
  // The first iteration after a reset is the reset itself, which still
  // reports the wind as it was; the replay starts on the next one.
  ASSERT_GT(world.series.size(), 400u);
  for (std::size_t i = 1; i < world.series.size(); ++i)
    ASSERT_EQ(before[i - 1], world.series[i]) << "step " << i;
}

/////////////////////////////////////////////////
/// The topic turns gusts on in a world that has none, and off again.
TEST(WindGusts, TurnedOnAndOffOnTheTopic)
{
  WindWorld world("windfield.sdf");
  ASSERT_TRUE(world.Run(200));
  EXPECT_NEAR(0.0, SpeedSd(world.series), 1e-9) << "steady before";

  ASSERT_TRUE(SetWind("windfield", "speed_gust", 1.0));
  world.series.clear();
  ASSERT_TRUE(world.Run(5000));
  EXPECT_GT(SpeedSd(world.series), 0.3) << "gusting after";

  ASSERT_TRUE(SetWind("windfield", "speed_gust", 0.0));
  ASSERT_TRUE(world.Run(2));
  world.series.clear();
  ASSERT_TRUE(world.Run(200));
  EXPECT_NEAR(0.0, SpeedSd(world.series), 1e-9) << "steady again";
  EXPECT_NEAR(5.0, world.state.entity.Length(), 1e-9) << "back to the mean";
}

/////////////////////////////////////////////////
/// With a roughness length the wind falls off towards the water: 1 m up it
/// is ln(1 / z0) / ln(10 / z0) of the wind at the 10 m reference height,
/// which the wind entity keeps. A zero roughness length, sent on the topic,
/// is a uniform wind.
TEST(WindProfile, WeakerNearTheWater)
{
  WindWorld world("profile.sdf");
  ASSERT_TRUE(world.Run(10));
  const double z0 = 0.0002;
  EXPECT_NEAR(5.0, world.state.entity.X(), 1e-9);
  EXPECT_NEAR(5.0, world.state.sampled.X(), 1e-9);
  EXPECT_NEAR(5.0 * std::log(1.0 / z0) / std::log(10.0 / z0),
              world.state.low.X(), 1e-9);

  ASSERT_TRUE(SetWind("profile", "roughness_length", 0.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(5.0, world.state.low.X(), 1e-9);
}

/////////////////////////////////////////////////
/// The windage asks the wind at each shape: a box 1 m above the water, in
/// ln(1 / z0) / ln(10 / z0) of the wind, takes that squared of the force
/// a box at the 10 m reference height takes.
TEST(WindProfile, WindageTakesTheWindAtEachShape)
{
  TestFixture fixture(World("profile.sdf"));
  BoxState high;
  BoxState low;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    high = ReadBox(_ecm, "high_box");
    low = ReadBox(_ecm, "low_box");
  });
  fixture.Finalize();
  ASSERT_TRUE(fixture.Server()->Run(true, 500, false));

  ASSERT_TRUE(high.tracked && low.tracked);
  const double k = std::log(1.0 / 0.0002) / std::log(10.0 / 0.0002);
  EXPECT_NEAR(kSpeedAfterOneSecond, high.vel.X(), 0.01);
  EXPECT_NEAR(k * k, low.vel.X() / high.vel.X(), 0.03);
}

/////////////////////////////////////////////////
/// In a world turned 90 degrees against north, a wind from the west pushes
/// a box east in geographic terms: its longitude grows and its latitude
/// stays.
TEST(WindDirection, PushesGeographicEast)
{
  TestFixture fixture(World("heading.sdf"));
  BoxState box;
  std::optional<math::SphericalCoordinates> sc;
  fixture.OnPostUpdate([&](const UpdateInfo &,
      const EntityComponentManager &_ecm)
  {
    box = ReadBox(_ecm, "marked_box");
    const Entity world = _ecm.EntityByComponents(components::World());
    if (const auto *c =
        _ecm.Component<components::SphericalCoordinates>(world))
    {
      sc = c->Data();
    }
  });
  fixture.Finalize();
  ASSERT_TRUE(fixture.Server()->Run(true, 1000, false));

  ASSERT_TRUE(sc.has_value());
  ASSERT_TRUE(box.found);
  const auto start = sc->SphericalFromLocalPosition(
      math::CoordinateVector3::Metric(0.0, 0.0, 0.5));
  const auto end = sc->SphericalFromLocalPosition(
      math::CoordinateVector3::Metric(box.pos));
  ASSERT_TRUE(start.has_value() && end.has_value());
  EXPECT_GT(end->Lon()->Degree() - start->Lon()->Degree(), 1e-7) << "east";
  EXPECT_NEAR(start->Lat()->Degree(), end->Lat()->Degree(), 1e-9)
      << "not north or south";
}
