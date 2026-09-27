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

#include <gz/msgs/param.pb.h>
#include <gz/msgs/wind.pb.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

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

  /// \brief The wind asked through the recipe at the world's origin.
  public: math::Vector3d sampled;

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
  state.sampled = wind::WindAt(_ecm, math::Vector3d::Zero, _info.simTime);
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

  /// \brief The wind after the last step.
  public: WindState state;
};
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
