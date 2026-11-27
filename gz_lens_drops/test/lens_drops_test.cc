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

// The lens drops system in a world that does not render
// (test/worlds/lens_drops.sdf): a dry lens stays dry, rain leaves drops,
// the set topic starts the rain, wipes the lens, ignores bad values and
// refuses bad drops, a seeded run repeats, paused steps change nothing, a
// reset wipes the lens, and the system refuses a sensor that is not a plain
// camera. In test/worlds/lens_drops_pose.sdf, what the world does to a
// lens: gravity, rain, wind, and the camera's own motion.
// Nothing renders, so the render pass is never added; the camera test
// covers that.

#include <gtest/gtest.h>

#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/param.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/pose.pb.h>

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/transport/Node.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief The topics of a model's camera.
/// \param[in] _model The model.
/// \param[in] _leaf `set` or `state`.
/// \return The topic.
std::string Topic(const std::string &_model, const std::string &_leaf)
{
  return "/world/lens_drops_test/model/" + _model +
      "/link/link/sensor/camera/lens_drops/" + _leaf;
}

/// \brief The last state a lens published.
class State
{
  /// \brief Subscribe.
  /// \param[in] _topic The state topic.
  public: explicit State(const std::string &_topic)
  {
    this->node.Subscribe(_topic, &State::OnState, this);
  }

  /// \brief True once a state came.
  /// \return Whether one came.
  public: bool Seen()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->seen;
  }

  /// \brief Drops on the lens.
  /// \return The count, or -1 before a state came.
  public: int Count()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->seen ? this->count : -1;
  }

  /// \brief A parameter.
  /// \param[in] _key Its key.
  /// \return Its value, or NaN before a state came.
  public: double Value(const std::string &_key)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    const auto it = this->values.find(_key);
    return it == this->values.end() ?
        std::numeric_limits<double>::quiet_NaN() : it->second;
  }

  /// \brief The rain rate.
  /// \return mm/h.
  public: double Rain()
  {
    return this->Value("rain_rate");
  }

  /// \brief Whether the render pass is on the camera.
  /// \return The flag.
  public: bool Attached()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->attached;
  }

  /// \brief Simulation time of the last state.
  /// \return Seconds, or -1 before one came.
  public: double Stamp()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->seen ? this->stamp : -1.0;
  }

  /// \brief Wait for a state stamped at or after a time.
  /// \param[in] _time Seconds of simulation time.
  /// \return True if one came within 5 s of wall time.
  public: bool WaitFor(double _time)
  {
    for (int i = 0; i < 250; ++i)
    {
      if (this->Stamp() >= _time - 1e-9)
        return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

  /// \brief A state came.
  /// \param[in] _msg The state.
  private: void OnState(const msgs::Param &_msg)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->seen = true;
    this->stamp = _msg.header().stamp().sec() +
        _msg.header().stamp().nsec() * 1e-9;
    for (const auto &[key, value] : _msg.params())
    {
      if (value.type() == msgs::Any::DOUBLE)
        this->values[key] = value.double_value();
      else if (key == "count")
        this->count = value.int_value();
      else if (key == "attached")
        this->attached = value.bool_value();
    }
  }

  /// \brief Guards the rest.
  private: std::mutex mutex;

  /// \brief True once a state came.
  private: bool seen{false};

  /// \brief The last state's values.
  private: std::map<std::string, double> values;

  /// \brief The last state's count.
  private: int count{0};

  /// \brief The last state's attached flag.
  private: bool attached{false};

  /// \brief The last state's stamp, in seconds.
  private: double stamp{0.0};

  /// \brief Transport node. Last, so it is destroyed first.
  private: transport::Node node;
};

/// \brief Publish one set message.
/// \param[in] _topic The set topic.
/// \param[in] _values Keys and numbers.
/// \return True if someone was subscribed.
bool Set(const std::string &_topic,
         const std::map<std::string, double> &_values)
{
  transport::Node node;
  auto pub = node.Advertise<msgs::Param>(_topic);
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (!pub.HasConnections())
    return false;
  msgs::Param msg;
  for (const auto &[key, value] : _values)
  {
    msgs::Any &any = (*msg.mutable_params())[key];
    any.set_type(msgs::Any::DOUBLE);
    any.set_double_value(value);
  }
  // Let the subscription settle on both sides before the one message goes.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pub.Publish(msg);
  return true;
}

/// \brief A server on the test world.
/// \return Its configuration.
ServerConfig Config()
{
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, "lens_drops.sdf"));
  return config;
}

/// \brief Run a server some steps, blocking.
/// \param[in,out] _server The server.
/// \param[in] _steps How many.
void Advance(Server &_server, int _steps)
{
  _server.Run(true, static_cast<std::uint64_t>(_steps), false);
}
/// \brief Publish one set message of any values.
/// \param[in] _topic The set topic.
/// \param[in] _msg The message.
/// \return True if someone was subscribed.
bool SetRaw(const std::string &_topic, const msgs::Param &_msg)
{
  transport::Node node;
  auto pub = node.Advertise<msgs::Param>(_topic);
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  if (!pub.HasConnections())
    return false;
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  pub.Publish(_msg);
  return true;
}

/// \brief Step until a state stamped at or after a time came.
/// \param[in,out] _server The server.
/// \param[in,out] _state The state.
/// \param[in] _time Seconds of simulation time.
/// \return True if it came.
bool StepUntil(Server &_server, State &_state, double _time)
{
  const auto it = _server.IterationCount();
  const double now = it ? static_cast<double>(*it) * 1e-3 : 0.0;
  if (_time > now)
    Advance(_server, static_cast<int>(std::ceil((_time - now) * 1e3)) + 1);
  return _state.WaitFor(_time);
}
}  // namespace

//////////////////////////////////////////////////
TEST(LensDropsSystem, DryLensStaysDry)
{
  State dry(Topic("dry", "state"));
  Server server(Config());
  Advance(server, 1000);
  ASSERT_TRUE(dry.WaitFor(0.8));
  EXPECT_EQ(dry.Count(), 0);
  EXPECT_DOUBLE_EQ(dry.Rain(), 0.0);
  EXPECT_FALSE(dry.Attached());
}

//////////////////////////////////////////////////
TEST(LensDropsSystem, RainLeavesDrops)
{
  State wet(Topic("wet", "state"));
  Server server(Config());
  Advance(server, 3000);
  ASSERT_TRUE(wet.WaitFor(2.8));
  EXPECT_GE(wet.Count(), 3);
  EXPECT_DOUBLE_EQ(wet.Rain(), 200.0);
  EXPECT_FALSE(wet.Attached());
}

//////////////////////////////////////////////////
TEST(LensDropsSystem, SetTopicStartsAndClears)
{
  State dry(Topic("dry", "state"));
  Server server(Config());
  Advance(server, 10);
  ASSERT_TRUE(Set(Topic("dry", "set"), {{"rain_rate", 400.0}}));

  bool wet = false;
  for (int i = 0; i < 50 && !wet; ++i)
  {
    Advance(server, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    wet = dry.Count() >= 3;
  }
  ASSERT_TRUE(wet) << "the rain on the set topic left no drops";
  EXPECT_DOUBLE_EQ(dry.Rain(), 400.0);

  ASSERT_TRUE(Set(Topic("dry", "set"), {{"rain_rate", 0.0}, {"clear", 1.0}}));
  bool clear = false;
  for (int i = 0; i < 50 && !clear; ++i)
  {
    Advance(server, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    clear = dry.Count() == 0;
  }
  EXPECT_TRUE(clear) << "clear left drops on the lens";
  EXPECT_DOUBLE_EQ(dry.Rain(), 0.0);
}

//////////////////////////////////////////////////
TEST(LensDropsSystem, IgnoresBadValues)
{
  State dry(Topic("dry", "state"));
  Server server(Config());
  Advance(server, 10);

  transport::Node node;
  auto pub = node.Advertise<msgs::Param>(Topic("dry", "set"));
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(pub.HasConnections());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  msgs::Param msg;
  msgs::Any &rain = (*msg.mutable_params())["rain_rate"];
  rain.set_type(msgs::Any::DOUBLE);
  rain.set_double_value(std::numeric_limits<double>::quiet_NaN());
  msgs::Any &colour = (*msg.mutable_params())["colour"];
  colour.set_type(msgs::Any::DOUBLE);
  colour.set_double_value(1.0);
  msgs::Any &exposure = (*msg.mutable_params())["exposure"];
  exposure.set_type(msgs::Any::DOUBLE);
  exposure.set_double_value(0.5);
  pub.Publish(msg);

  bool applied = false;
  for (int i = 0; i < 50 && !applied; ++i)
  {
    Advance(server, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    applied = dry.Value("exposure") == 0.5;
  }
  EXPECT_TRUE(applied) << "a good key beside bad ones was not applied";
  EXPECT_DOUBLE_EQ(dry.Rain(), 0.0);
}

//////////////////////////////////////////////////
TEST(LensDropsSystem, SeededRunRepeats)
{
  // States go out every 0.2 s from the first step: the last one in 2 s of
  // 1 ms steps is stamped 1.801 s.
  int first{-1};
  {
    State wet(Topic("wet", "state"));
    Server server(Config());
    Advance(server, 2000);
    ASSERT_TRUE(wet.WaitFor(1.801));
    first = wet.Count();
  }
  ASSERT_GT(first, 0);

  State wet(Topic("wet", "state"));
  Server server(Config());
  Advance(server, 2000);
  ASSERT_TRUE(wet.WaitFor(1.801));
  EXPECT_EQ(wet.Count(), first);
}

//////////////////////////////////////////////////
TEST(LensDropsSystem, NotOnAPlainCamera)
{
  Server server(Config());
  Advance(server, 500);
  transport::Node node;
  std::vector<std::string> topics;
  node.TopicList(topics);
  bool found = false;
  for (const std::string &topic : topics)
  {
    EXPECT_EQ(topic.find("/model/wrong/"), std::string::npos) << topic;
    EXPECT_EQ(topic.find("/model/depth/link/link/sensor/camera/lens_drops"),
              std::string::npos) << topic;
    found = found || topic.find("/model/wet/") != std::string::npos;
  }
  EXPECT_TRUE(found) << "the cameras' topics were not listed either";
}

namespace
{
/// \brief The topics of a camera in the pose world.
/// \param[in] _model The model.
/// \param[in] _leaf `set` or `state`.
/// \return The topic.
std::string PoseTopic(const std::string &_model, const std::string &_leaf)
{
  return "/world/lens_drops_pose/model/" + _model +
      "/link/link/sensor/camera/lens_drops/" + _leaf;
}

/// \brief A server on the pose world.
/// \return Its configuration.
ServerConfig PoseConfig()
{
  ServerConfig config;
  config.SetSdfFile(common::joinPaths(TEST_WORLD_DIR, "lens_drops_pose.sdf"));
  return config;
}
}  // namespace

//////////////////////////////////////////////////
TEST(LensDropsEnvironment, LevelCameraFeelsGravityDown)
{
  State level(PoseTopic("level", "state"));
  Server server(PoseConfig());
  Advance(server, 500);
  ASSERT_TRUE(level.WaitFor(0.401));
  EXPECT_NEAR(level.Value("gravity_x"), 0.0, 0.01);
  EXPECT_NEAR(level.Value("gravity_y"), 1.0, 0.01);
  EXPECT_NEAR(level.Value("flux"), 0.0, 1e-6);
}

//////////////////////////////////////////////////
TEST(LensDropsEnvironment, RolledCameraFeelsGravitySideways)
{
  State rolled(PoseTopic("rolled", "state"));
  Server server(PoseConfig());
  Advance(server, 500);
  ASSERT_TRUE(rolled.WaitFor(0.401));
  EXPECT_NEAR(std::abs(rolled.Value("gravity_x")), 1.0, 0.01);
  EXPECT_NEAR(rolled.Value("gravity_y"), 0.0, 0.01);
}

//////////////////////////////////////////////////
TEST(LensDropsEnvironment, UpCameraCatchesRain)
{
  State up(PoseTopic("up", "state"));
  Server server(PoseConfig());
  Advance(server, 500);
  ASSERT_TRUE(up.WaitFor(0.401));
  EXPECT_NEAR(up.Value("flux"), 1.0, 0.01);
  EXPECT_LT(std::hypot(up.Value("gravity_x"), up.Value("gravity_y")), 0.01);
}

//////////////////////////////////////////////////
TEST(LensDropsEnvironment, WindBringsRain)
{
  // 20 m/s at 10 m coming from +x, blowing into the level camera's lens;
  // lower at its 1 m, but still well over the rain's fall speed.
  State level(PoseTopic("level", "state"));
  Server server(PoseConfig());
  Advance(server, 10);
  ASSERT_TRUE(Set("/world/lens_drops_pose/wind/set",
      {{"speed", 20.0}, {"direction", 90.0}}));
  bool wet = false;
  for (int i = 0; i < 20 && !wet; ++i)
  {
    Advance(server, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    wet = level.Value("flux") > 0.5;
  }
  EXPECT_TRUE(wet) << "flux " << level.Value("flux");
}

//////////////////////////////////////////////////
// Paused steps change nothing: a run with a pause in it ends with the
// drops of the same run without one.
TEST(LensDropsSystem, PausedStepsChangeNothing)
{
  int straight{-1};
  {
    State wet(Topic("wet", "state"));
    Server server(Config());
    Advance(server, 2000);
    ASSERT_TRUE(wet.WaitFor(1.801));
    straight = wet.Count();
  }
  ASSERT_GT(straight, 0);

  State wet(Topic("wet", "state"));
  Server server(Config());
  Advance(server, 1000);
  // Paused iterations do not count towards Run's, so step them one by one
  for (int i = 0; i < 300; ++i)
    ASSERT_TRUE(server.RunOnce(true));
  Advance(server, 1000);
  ASSERT_TRUE(wet.WaitFor(1.801));
  EXPECT_EQ(wet.Count(), straight);
}

//////////////////////////////////////////////////
// A world reset wipes the lens and starts the seed again.
TEST(LensDropsSystem, ResetWipesAndRepeats)
{
  State wet(Topic("wet", "state"));
  Server server(Config());
  Advance(server, 2000);
  ASSERT_TRUE(wet.WaitFor(1.801));
  const int first = wet.Count();
  ASSERT_GT(first, 0);

  server.ResetAll();
  Advance(server, 1);
  Advance(server, 2000);
  // The reset run's state at 1.801 s comes after a second one stamped so
  for (int i = 0; i < 250 && wet.Stamp() > 1.9; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(wet.WaitFor(1.801));
  EXPECT_LE(wet.Stamp(), 2.1) << "the reset did not reset the time";
  EXPECT_EQ(wet.Count(), first);
}

//////////////////////////////////////////////////
// A drop put on the lens is checked: one out of range, not finite, or
// without all three keys is refused; a good one lands.
TEST(LensDropsSystem, AddChecksTheDrop)
{
  State dry(Topic("dry", "state"));
  Server server(Config());
  Advance(server, 10);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::map<std::string, double>> bad{
      {{"add_x", 0.5}, {"add_y", 0.5}},
      {{"add_x", 0.5}, {"add_y", 0.5}, {"add_radius", 0.0}},
      {{"add_x", 0.5}, {"add_y", 0.5}, {"add_radius", 0.5}},
      {{"add_x", 1.5}, {"add_y", 0.5}, {"add_radius", 0.05}},
      {{"add_x", -0.1}, {"add_y", 0.5}, {"add_radius", 0.05}},
      {{"add_x", 0.5}, {"add_y", 1e300}, {"add_radius", 0.05}},
      {{"add_x", nan}, {"add_y", 0.5}, {"add_radius", 0.05}}};
  for (const auto &values : bad)
  {
    msgs::Param msg;
    for (const auto &[key, value] : values)
    {
      msgs::Any &any = (*msg.mutable_params())[key];
      any.set_type(msgs::Any::DOUBLE);
      any.set_double_value(value);
    }
    ASSERT_TRUE(SetRaw(Topic("dry", "set"), msg));
  }
  double t = 0.2;
  for (; t < 2.0 && !(dry.Count() > 0); t += 0.2)
    ASSERT_TRUE(StepUntil(server, dry, t));
  EXPECT_EQ(dry.Count(), 0) << "a bad drop landed";

  ASSERT_TRUE(Set(Topic("dry", "set"),
      {{"add_x", 0.5}, {"add_y", 0.5}, {"add_radius", 0.05}}));
  bool landed = false;
  for (int i = 0; i < 20 && !landed; ++i, t += 0.2)
  {
    ASSERT_TRUE(StepUntil(server, dry, t));
    landed = dry.Count() == 1;
  }
  EXPECT_TRUE(landed);
}

//////////////////////////////////////////////////
// <wind>false: the wind that drives rain into one camera's lens does not
// reach the camera beside it.
TEST(LensDropsEnvironment, WindOffKeepsStillAir)
{
  State level(PoseTopic("level", "state"));
  State still(PoseTopic("still_air", "state"));
  Server server(PoseConfig());
  Advance(server, 10);
  ASSERT_TRUE(Set("/world/lens_drops_pose/wind/set",
      {{"speed", 20.0}, {"direction", 90.0}}));
  bool wet = false;
  for (int i = 0; i < 20 && !wet; ++i)
  {
    Advance(server, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    wet = level.Value("flux") > 0.5;
  }
  ASSERT_TRUE(wet) << "flux " << level.Value("flux");
  ASSERT_TRUE(still.WaitFor(level.Stamp()));
  EXPECT_NEAR(still.Value("flux"), 0.0, 1e-6);
}

//////////////////////////////////////////////////
// A camera driven forward runs into the rain: at 5 m/s level, rain falling
// at 6.5 m/s comes into the lens at 5 / 6.5 of its fall speed. A teleport
// is no motion: the flux stays what the speed makes it.
TEST(LensDropsEnvironment, MovingCameraRunsIntoRain)
{
  State mover(PoseTopic("mover", "state"));
  Server server(PoseConfig());
  Advance(server, 10);

  // Teleported 50 m ahead while still: no flux
  transport::Node node;
  msgs::Pose pose;
  pose.set_name("mover");
  pose.mutable_position()->set_x(50.0);
  pose.mutable_position()->set_y(8.0);
  pose.mutable_position()->set_z(1.0);
  pose.mutable_orientation()->set_w(1.0);
  std::function<void(const msgs::Boolean &, const bool)> done =
      [](const msgs::Boolean &, const bool) {};
  const bool requested =
      node.Request<msgs::Pose, msgs::Boolean>(
          "/world/lens_drops_pose/set_pose", pose, done);
  ASSERT_TRUE(requested);
  for (int i = 0; i < 10; ++i)
  {
    Advance(server, 100);
    EXPECT_LT(mover.Value("flux"), 1e-6) << "the teleport made rain";
  }

  auto pub = node.Advertise<msgs::Twist>("/model/mover/cmd_vel");
  for (int i = 0; i < 100 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(pub.HasConnections());
  msgs::Twist twist;
  twist.mutable_linear()->set_x(5.0);
  pub.Publish(twist);
  bool moving = false;
  for (int i = 0; i < 30 && !moving; ++i)
  {
    Advance(server, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    moving = std::abs(mover.Value("flux") - 5.0 / 6.5) < 0.02;
  }
  EXPECT_TRUE(moving) << "flux " << mover.Value("flux");
  // Moving straight ahead: no air across the lens beyond the headwind, and
  // gravity down the image
  EXPECT_NEAR(mover.Value("gravity_y"), 1.0, 0.01);
}

//////////////////////////////////////////////////
// A camera spawned at run time, as every vehicle enters a world: its
// topics carry the world's name, and its rain lands.
TEST(LensDropsSystem, SpawnedCameraIsNamedUnderTheWorld)
{
  State spawned(Topic("spawned", "state"));
  Server server(Config());
  Advance(server, 10);
  msgs::EntityFactory factory;
  factory.set_sdf(R"(<?xml version="1.0"?>
<sdf version="1.9">
  <model name="spawned">
    <static>true</static>
    <pose>0 8 0 0 -1.5707963 0</pose>
    <link name="link">
      <sensor name="camera" type="camera">
        <camera>
          <horizontal_fov>1.2</horizontal_fov>
          <image><width>160</width><height>120</height></image>
        </camera>
        <plugin filename="gz-maritime-lens-drops-system"
                name="gz::sim::maritime::LensDrops">
          <rain_rate>200</rain_rate>
          <dry_time>3600</dry_time>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>)");
  transport::Node node;
  std::function<void(const msgs::Boolean &, const bool)> done =
      [](const msgs::Boolean &, const bool) {};
  ASSERT_TRUE((node.Request<msgs::EntityFactory, msgs::Boolean>(
      "/world/lens_drops_test/create", factory, done)));
  bool wet = false;
  for (int i = 0; i < 20 && !wet; ++i)
  {
    Advance(server, 200);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    wet = spawned.Count() >= 3;
  }
  EXPECT_TRUE(wet) << "no state with drops under the world's name";
  std::vector<std::string> topics;
  node.TopicList(topics);
  for (const std::string &topic : topics)
  {
    EXPECT_NE(topic.rfind("/model/spawned/", 0), 0u)
        << topic << ": a topic without the world's name";
  }
}

//////////////////////////////////////////////////
// The world's gravity, not a fixed one: (3, 2, -4) m/s^2 on a level
// camera is -2 / g to the image's right and 4 / g down it.
TEST(LensDropsEnvironment, GravityIsTheWorlds)
{
  State level("/world/lens_drops_gravity/model/level/link/link/sensor/"
              "camera/lens_drops/state");
  ServerConfig config;
  config.SetSdfFile(
      common::joinPaths(TEST_WORLD_DIR, "lens_drops_gravity.sdf"));
  Server server(config);
  Advance(server, 500);
  ASSERT_TRUE(level.WaitFor(0.401));
  EXPECT_NEAR(level.Value("gravity_x"), -2.0 / 9.80665, 1e-3);
  EXPECT_NEAR(level.Value("gravity_y"), 4.0 / 9.80665, 1e-3);
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  const std::string partition =
      "gz_lens_drops_test_" + std::to_string(::getpid());
  ::setenv("GZ_PARTITION", partition.c_str(), 1);
  return RUN_ALL_TESTS();
}
