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

#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>

#include "gz/sim/components/Windfield.hh"
#include "gz/sim/wind/WindSampler.hh"

using namespace gz;
using namespace sim;

namespace
{
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
}  // namespace

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
