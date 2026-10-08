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
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <gz/common/Filesystem.hh>
#include <gz/math/CoordinateVector3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/TestFixture.hh>
#include <gz/sim/components/SphericalCoordinates.hh>
#include <gz/sim/components/World.hh>

#include "gz/sim/components/OceanCurrentfield.hh"
#include "gz/sim/ocean_current/OceanCurrentModel.hh"
#include "gz/sim/ocean_current/OceanCurrentSampler.hh"

using namespace gz;
using namespace sim;

namespace
{
/// \brief What one iteration of the world tells us about its current.
struct CurrentState
{
  /// \brief The current asked through the recipe at the world's origin, by
  /// a consumer that has been there since the start.
  public: math::Vector3d sampled;

  /// \brief The current asked somewhere else, a kilometre away and deep.
  public: math::Vector3d elsewhere;

  /// \brief The current read by a sampler made this very step, the way a
  /// system on a vehicle spawned now would read it.
  public: math::Vector3d late;

  /// \brief The recipe, if the world has one.
  public: std::optional<ocean_current::OceanCurrentfieldData> recipe;

  /// \brief The world's spherical coordinates, if it has them.
  public: std::optional<math::SphericalCoordinates> sc;
};

/// \brief Read the current out of the ECM.
/// \param[in] _info Update info.
/// \param[in] _ecm Entity component manager.
/// \return The current this iteration.
CurrentState ReadCurrent(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  CurrentState state;
  state.sampled = ocean_current::OceanCurrentAt(_ecm, {0, 0, 0},
                                                _info.simTime);
  state.elsewhere = ocean_current::OceanCurrentAt(_ecm, {1000, -700, -30},
                                                  _info.simTime);
  ocean_current::OceanCurrentSampler fresh;
  fresh.Sync(_ecm);
  state.late = fresh.At({0, 0, 0}, _info.simTime);
  const Entity world = _ecm.EntityByComponents(components::World());
  if (const auto *field = _ecm.Component<components::OceanCurrentfield>(world))
    state.recipe = field->Data();
  if (const auto *sc = _ecm.Component<components::SphericalCoordinates>(world))
    state.sc = sc->Data();
  return state;
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

/// \brief Publish one message on the world's ocean current topic.
/// \param[in] _world World name.
/// \param[in] _msg The message.
/// \return True once the message was sent to a subscriber.
bool Publish(const std::string &_world, const msgs::Param &_msg)
{
  static transport::Node node;
  auto pub = node.Advertise<msgs::Param>(
      "/world/" + _world + "/ocean_current/set");
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (!pub.HasConnections())
    return false;
  const bool sent = pub.Publish(_msg);
  // Delivery is asynchronous; give the subscriber a moment to queue it.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return sent;
}

/// \brief Ask the ocean current system to change a numeric parameter.
/// \param[in] _world World name.
/// \param[in] _key Parameter name.
/// \param[in] _value Its new value.
/// \return True once the message was sent to a subscriber.
bool SetCurrent(const std::string &_world, const std::string &_key,
                double _value)
{
  msgs::Param msg;
  auto &any = (*msg.mutable_params())[_key];
  any.set_type(msgs::Any::DOUBLE);
  any.set_double_value(_value);
  return Publish(_world, msg);
}

/// \brief Ask the ocean current system to change the source.
/// \param[in] _world World name.
/// \param[in] _source The new source.
/// \return True once the message was sent to a subscriber.
bool SetSource(const std::string &_world, const std::string &_source)
{
  msgs::Param msg;
  auto &any = (*msg.mutable_params())["source"];
  any.set_type(msgs::Any::STRING);
  any.set_string_value(_source);
  return Publish(_world, msg);
}

/// \brief Path to one of the test worlds.
/// \param[in] _file World file name.
/// \return Its absolute path.
std::string World(const std::string &_file)
{
  return common::joinPaths(TEST_WORLD_DIR, _file);
}

/// \brief Load a world and keep its current up to date.
class CurrentWorld
{
  /// \brief Constructor.
  /// \param[in] _file World file name.
  public: explicit CurrentWorld(const std::string &_file)
    : fixture(World(_file))
  {
    this->fixture.OnPostUpdate([this](const UpdateInfo &_info,
        const EntityComponentManager &_ecm)
    {
      this->state = ReadCurrent(_info, _ecm);
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

  /// \brief The current after the last step.
  public: CurrentState state;
};

/// \brief The last message on a topic.
/// \tparam MsgT Message type.
template <typename MsgT>
class Last
{
  /// \brief Constructor: subscribe.
  /// \param[in] _topic Topic.
  public: explicit Last(const std::string &_topic)
  {
    this->node.Subscribe(_topic, std::function<void(const MsgT &)>(
        [this](const MsgT &_msg)
        {
          const std::lock_guard<std::mutex> lock(this->mutex);
          this->last = _msg;
        }));
  }

  /// \brief Wait up to a second for a message and return the last one.
  /// \return The message, if one came.
  public: std::optional<MsgT> Get()
  {
    for (int i = 0; i < 50; ++i)
    {
      {
        const std::lock_guard<std::mutex> lock(this->mutex);
        if (this->last)
          return this->last;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->last;
  }

  /// \brief Transport node.
  private: transport::Node node;

  /// \brief Guards the message.
  private: std::mutex mutex;

  /// \brief The last message.
  private: std::optional<MsgT> last;
};

/// \brief The frame id in a header, if any.
/// \param[in] _header The header.
/// \return The frame id, or an empty string.
std::string FrameId(const msgs::Header &_header)
{
  for (const auto &data : _header.data())
  {
    if (data.key() == "frame_id" && data.value_size() > 0)
      return data.value(0);
  }
  return "";
}

/// \brief A model that returns an upward current of the recipe's speed: a
/// new model, registered by a test here the way a plugin would, that the
/// ocean current system knows nothing about.
class Upwelling : public ocean_current::IOceanCurrentModel
{
  public: void SetParameters(const ocean_current::OceanCurrentParameters &_p)
      override
  {
    this->p = _p;
  }
  public: math::Vector3d Velocity(const math::Vector3d &, double) const
      override
  {
    return {0.0, 0.0, this->p.speed};
  }
  public: std::string_view Kind() const override { return "upwelling"; }
  private: ocean_current::OceanCurrentParameters p;
};
}  // namespace

/////////////////////////////////////////////////
/// <speed> and <direction> set the current: 90, setting east, flows towards
/// +x, the same everywhere. The recipe sits on the world, naming the
/// standard model, with a seed and no source.
TEST(OceanCurrentField, SpeedAndDirectionFromTheWorld)
{
  CurrentWorld world("ocean_currentfield.sdf");
  ASSERT_TRUE(world.Run(10));
  EXPECT_NEAR(1.0, world.state.sampled.X(), 1e-9);
  EXPECT_NEAR(0.0, world.state.sampled.Y(), 1e-9);
  EXPECT_NEAR(0.0, world.state.sampled.Z(), 1e-9);
  EXPECT_EQ(world.state.sampled, world.state.elsewhere)
      << "the current is uniform";
  ASSERT_TRUE(world.state.recipe.has_value());
  EXPECT_EQ("standard", world.state.recipe->model);
  EXPECT_NEAR(1.0, world.state.recipe->params.speed, 1e-9);
  EXPECT_NEAR(90.0, world.state.recipe->params.direction, 1e-9);
  EXPECT_NE(0u, world.state.recipe->params.seed)
      << "a 0 seed is resolved before the recipe is written";
  EXPECT_TRUE(world.state.recipe->params.source.empty());
}

/////////////////////////////////////////////////
/// A consumer that arrives late, a system on a vehicle spawned after the
/// world has run a while, reads the same current as one loaded with the
/// world: the recipe is on the world entity, not in anyone's startup.
TEST(OceanCurrentField, LateConsumerReadsTheRecipe)
{
  CurrentWorld world("ocean_currentfield.sdf");
  ASSERT_TRUE(world.Run(500));
  EXPECT_EQ(world.state.sampled, world.state.late);
  EXPECT_NEAR(1.0, world.state.late.X(), 1e-9);
  EXPECT_NEAR(0.0, world.state.late.Y(), 1e-9);
}

/////////////////////////////////////////////////
/// The current is constant for the run: an hour later it is what the world
/// file said, under the same recipe.
TEST(OceanCurrentField, ConstantUntilChanged)
{
  CurrentWorld world("ocean_currentfield.sdf");
  ASSERT_TRUE(world.Run(1));
  ASSERT_TRUE(world.state.recipe.has_value());
  const auto generation = world.state.recipe->generation;
  // The default step is 1 ms; 3600 steps stand in for the hour, since
  // nothing in the recipe depends on how far the clock went.
  ASSERT_TRUE(world.Run(3600));
  EXPECT_NEAR(1.0, world.state.sampled.X(), 1e-9);
  EXPECT_NEAR(0.0, world.state.sampled.Y(), 1e-9);
  EXPECT_EQ(generation, world.state.recipe->generation)
      << "nothing rewrote the recipe";
}

/////////////////////////////////////////////////
/// The topic changes the current while the world runs, as a new recipe that
/// a consumer reads on the next step; an unknown key or an out of range
/// value changes nothing.
TEST(OceanCurrentField, ChangedAtRunTimeOnItsTopic)
{
  CurrentWorld world("ocean_currentfield.sdf");
  ASSERT_TRUE(world.Run(1));
  ASSERT_TRUE(world.state.recipe.has_value());
  const auto generation = world.state.recipe->generation;

  ASSERT_TRUE(SetCurrent("ocean_currentfield", "direction", 180.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(0.0, world.state.sampled.X(), 1e-9);
  EXPECT_NEAR(-1.0, world.state.sampled.Y(), 1e-9)
      << "setting south flows south";
  EXPECT_EQ(world.state.sampled, world.state.late)
      << "a consumer made now reads the change too";
  EXPECT_GT(world.state.recipe->generation, generation);

  ASSERT_TRUE(SetCurrent("ocean_currentfield", "speed", 0.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(0.0, world.state.sampled.Length(), 1e-9);

  const auto before = world.state.recipe->generation;
  ASSERT_TRUE(SetCurrent("ocean_currentfield", "gust", 3.0));
  ASSERT_TRUE(SetCurrent("ocean_currentfield", "speed", -1.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_EQ(before, world.state.recipe->generation);
  EXPECT_NEAR(0.0, world.state.sampled.Length(), 1e-9);
}

/////////////////////////////////////////////////
/// The source is a string key on the topic; it reaches the recipe as a new
/// generation and changes nothing the standard model reads.
TEST(OceanCurrentField, SourceOnTheTopic)
{
  CurrentWorld world("ocean_currentfield.sdf");
  ASSERT_TRUE(world.Run(1));
  ASSERT_TRUE(world.state.recipe.has_value());
  const auto generation = world.state.recipe->generation;

  ASSERT_TRUE(SetSource("ocean_currentfield", "noaa_blended_currents.nc"));
  ASSERT_TRUE(world.Run(2));
  EXPECT_EQ("noaa_blended_currents.nc", world.state.recipe->params.source);
  EXPECT_GT(world.state.recipe->generation, generation);
  EXPECT_NEAR(1.0, world.state.sampled.X(), 1e-9)
      << "the standard model reads no source";

  const auto before = world.state.recipe->generation;
  ASSERT_TRUE(SetCurrent("ocean_currentfield", "source", 4.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_EQ(before, world.state.recipe->generation);
}

/////////////////////////////////////////////////
/// A world names another model, registered by someone else, and the system
/// and the consumers run it unchanged.
TEST(OceanCurrentField, ModelFromTheWorldFile)
{
  ocean_current::RegisterOceanCurrentModelFactory("upwelling",
      [] { return std::make_unique<Upwelling>(); });
  CurrentWorld world("ocean_current_upwelling.sdf");
  ASSERT_TRUE(world.Run(10));
  ASSERT_TRUE(world.state.recipe.has_value());
  EXPECT_EQ("upwelling", world.state.recipe->model);
  EXPECT_NEAR(0.0, world.state.sampled.X(), 1e-9);
  EXPECT_NEAR(0.0, world.state.sampled.Y(), 1e-9);
  EXPECT_NEAR(0.3, world.state.sampled.Z(), 1e-9);
  EXPECT_EQ(world.state.sampled, world.state.late);
}

/////////////////////////////////////////////////
/// A reset brings back the world file's current, under a new generation,
/// so a consumer that cached anything refreshes it.
TEST(OceanCurrentField, ResetRewritesTheWorldsCurrent)
{
  CurrentWorld world("ocean_currentfield.sdf");
  ASSERT_TRUE(world.Run(1));
  ASSERT_TRUE(SetCurrent("ocean_currentfield", "speed", 2.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(2.0, world.state.sampled.X(), 1e-9);
  ASSERT_TRUE(world.state.recipe.has_value());
  const auto generation = world.state.recipe->generation;

  ASSERT_TRUE(ResetWorld("ocean_currentfield"));
  ASSERT_TRUE(world.Run(3));
  EXPECT_NEAR(1.0, world.state.sampled.X(), 1e-9)
      << "the world file's current, not the one set before the reset";
  EXPECT_NEAR(1.0, world.state.recipe->params.speed, 1e-9);
  EXPECT_EQ("standard", world.state.recipe->model);
  EXPECT_GT(world.state.recipe->generation, generation)
      << "the restored recipe is new to every consumer";

  ASSERT_TRUE(SetCurrent("ocean_currentfield", "direction", 180.0));
  ASSERT_TRUE(world.Run(2));
  EXPECT_NEAR(-1.0, world.state.sampled.Y(), 1e-9)
      << "a change after the reset starts from the world file's speed";
}

/////////////////////////////////////////////////
/// The current is published as ground truth, a twist in the world frame.
TEST(OceanCurrentField, GroundTruthIsPublished)
{
  Last<msgs::Twist> truth("/world/ocean_currentfield/ocean_current_info");
  CurrentWorld world("ocean_currentfield.sdf");
  ASSERT_TRUE(world.Run(200));

  const auto msg = truth.Get();
  ASSERT_TRUE(msg.has_value());
  EXPECT_NEAR(1.0, msg->linear().x(), 1e-9);
  EXPECT_NEAR(0.0, msg->linear().y(), 1e-9);
  EXPECT_EQ("world", FrameId(msg->header()));
}

/////////////////////////////////////////////////
/// North is the world's north, the one its spherical coordinates and GPS
/// use, not its +y axis: in a world turned 90 degrees, a current setting
/// east flows geographic east.
TEST(OceanCurrentDirection, NorthIsTheWorldsNorth)
{
  CurrentWorld world("heading.sdf");
  ASSERT_TRUE(world.Run(2));
  ASSERT_TRUE(world.state.sc.has_value());
  const auto expected = world.state.sc->LocalFromGlobalVelocity(
      math::CoordinateVector3::Metric(1.0, 0.0, 0.0));
  ASSERT_TRUE(expected.has_value());
  EXPECT_NEAR(expected->X().value(), world.state.sampled.X(), 1e-9);
  EXPECT_NEAR(expected->Y().value(), world.state.sampled.Y(), 1e-9);
  EXPECT_GT(std::abs(world.state.sampled.Y()), 0.99)
      << "the heading turned it off +x";
}
