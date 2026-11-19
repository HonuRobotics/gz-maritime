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
// the set topic starts the rain, wipes the lens and ignores bad values, a
// seeded run repeats, and the system refuses a sensor that is not a camera.
// Nothing renders, so the render pass is never added; the camera test
// covers that.

#include <gtest/gtest.h>

#include <gz/msgs/param.pb.h>

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
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
TEST(LensDropsSystem, NotOnACamera)
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
    found = found || topic.find("/model/wet/") != std::string::npos;
  }
  EXPECT_TRUE(found) << "the cameras' topics were not listed either";
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
