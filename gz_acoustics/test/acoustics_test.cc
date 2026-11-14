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
#include <gz/msgs/Utility.hh>

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <utility>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/common/TempDirectory.hh>
#include <gz/math/Helpers.hh>
#include <gz/transport/Node.hh>

#include <gz/sim/TestFixture.hh>

using namespace gz;
using namespace sim;

namespace
{
/// \brief The step of the test worlds, s.
constexpr double kStep{0.004};

/// \brief Full path of a test world.
/// \param[in] _file World file name, or an absolute path.
/// \return The path.
std::string World(const std::string &_file)
{
  // A path of its own, such as a copy in a temporary directory, as is.
  if (!_file.empty() && _file.front() == '/')
    return _file;
  return common::joinPaths(TEST_WORLD_DIR, _file);
}

/// \brief Steps for a span of simulation time.
/// \param[in] _seconds s.
/// \return Steps.
std::size_t Steps(double _seconds)
{
  return static_cast<std::size_t>(std::lround(_seconds / kStep));
}

/// \brief Every message published on a topic, in order.
class Readings
{
  /// \brief Subscribe.
  /// \param[in] _topic Topic.
  public: explicit Readings(const std::string &_topic)
  {
    this->node.Subscribe(_topic, &Readings::OnMsg, this);
  }

  /// \brief The messages so far.
  /// \return A copy.
  public: std::vector<msgs::Param> All()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    return this->msgs;
  }

  /// \brief Forget the messages so far.
  public: void Clear()
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->msgs.clear();
  }

  /// \brief Wait for messages in flight, then count them.
  /// \return The count.
  public: std::size_t Count()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return this->All().size();
  }

  /// \brief Handler.
  /// \param[in] _msg Message.
  private: void OnMsg(const msgs::Param &_msg)
  {
    const std::lock_guard<std::mutex> lock(this->mutex);
    this->msgs.push_back(_msg);
  }

  /// \brief Node.
  private: transport::Node node;

  /// \brief Guards the messages.
  private: std::mutex mutex;

  /// \brief The messages.
  private: std::vector<msgs::Param> msgs;
};

/// \brief A double from a message.
/// \param[in] _msg Message.
/// \param[in] _key Key.
/// \return The value, NaN when missing.
double Get(const msgs::Param &_msg, const std::string &_key)
{
  const auto it = _msg.params().find(_key);
  if (it == _msg.params().end())
    return NAN;
  return it->second.double_value();
}

/// \brief A boolean from a message.
/// \param[in] _msg Message.
/// \param[in] _key Key.
/// \return The value, false when missing.
bool GetBool(const msgs::Param &_msg, const std::string &_key)
{
  const auto it = _msg.params().find(_key);
  return it != _msg.params().end() && it->second.bool_value();
}

/// \brief Seconds of a header stamp.
/// \param[in] _msg Message.
/// \return s.
double Stamp(const msgs::Param &_msg)
{
  return _msg.header().stamp().sec() + 1e-9 * _msg.header().stamp().nsec();
}

/// \brief Send a gz.msgs.Param with one number.
/// \param[in] _topic Topic.
/// \param[in] _key Key.
/// \param[in] _value Value.
void Send(const std::string &_topic, const std::string &_key, double _value)
{
  transport::Node node;
  auto pub = node.Advertise<msgs::Param>(_topic);
  for (int i = 0; i < 50 && !pub.HasConnections(); ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  msgs::Param msg;
  auto &any = (*msg.mutable_params())[_key];
  any.set_type(msgs::Any::DOUBLE);
  any.set_double_value(_value);
  pub.Publish(msg);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

/// \brief A world under test.
class AcousticWorld
{
  /// \brief Load a world.
  /// \param[in] _file World file name.
  public: explicit AcousticWorld(const std::string &_file)
    : fixture(World(_file))
  {
    this->fixture.Finalize();
  }

  /// \brief Run a span of simulation time.
  /// \param[in] _seconds s.
  /// \return True on success.
  public: bool Run(double _seconds)
  {
    return this->fixture.Server()->Run(true, Steps(_seconds), false);
  }

  /// \brief The fixture.
  public: TestFixture fixture;
};
}

//////////////////////////////////////////////////
/// One reading per ping, from the pinger's direction in the receiver's
/// frame, after the travel time at the speed of sound.
TEST(Acoustics, OneReadingPerPingFromThePingersDirection)
{
  Readings listener("/listener");
  AcousticWorld world("pingers.sdf");
  ASSERT_TRUE(world.Run(5.0));
  const auto readings = listener.All();

  // Pings at 0, 1, 2, 3 and 4 s, each 0.1 s on its way.
  ASSERT_EQ(5u, readings.size());
  for (std::size_t i = 0; i < readings.size(); ++i)
  {
    const auto &r = readings[i];
    EXPECT_NEAR(GZ_PI / 2, Get(r, "bearing"), 1e-6);
    EXPECT_NEAR(0.0, Get(r, "elevation"), 1e-6);
    EXPECT_NEAR(150.0, Get(r, "range"), 1e-6);
    EXPECT_DOUBLE_EQ(30000.0, Get(r, "frequency"));
    EXPECT_NEAR(-20.0 * std::log10(150.0), Get(r, "amplitude"), 1e-6);
    EXPECT_TRUE(GetBool(r, "valid"));
    EXPECT_NEAR(static_cast<double>(i), Get(r, "emit_time"), 1e-9);
    // Heard at the first step the sound has covered the distance.
    EXPECT_GE(Stamp(r) - Get(r, "emit_time"), 0.1 - 1e-9);
    EXPECT_LT(Stamp(r) - Get(r, "emit_time"), 0.1 + kStep);
    ASSERT_GT(r.header().data_size(), 0);
    EXPECT_EQ("listener/receiver", r.header().data(0).value(0));
  }
}

//////////////////////////////////////////////////
/// Out of band, out of reach and out of the water, nothing is heard; a
/// receiver without range on reports none.
TEST(Acoustics, BandReachAndWater)
{
  Readings deaf("/deaf");
  Readings far("/far");
  Readings dry("/dry");
  AcousticWorld world("pingers.sdf");
  ASSERT_TRUE(world.Run(3.0));
  EXPECT_EQ(0u, deaf.Count());
  EXPECT_EQ(0u, far.Count());
  EXPECT_EQ(0u, dry.Count());
}

//////////////////////////////////////////////////
/// A receiver retuned on its set topic hears the band it moved to; a pinger
/// switched off on its set topic falls silent.
TEST(Acoustics, SetTopicsRetuneAndSwitchOff)
{
  Readings deaf("/deaf");
  Readings listener("/listener");
  AcousticWorld world("pingers.sdf");
  ASSERT_TRUE(world.Run(1.0));
  EXPECT_EQ(0u, deaf.Count());

  Send("/deaf/set", "frequency", 30000);
  ASSERT_TRUE(world.Run(2.0));
  EXPECT_EQ(2u, deaf.Count());
  for (const auto &r : deaf.All())
  {
    // 20 m north of the pinger, unturned: the pinger is to starboard.
    EXPECT_NEAR(-GZ_PI / 2, Get(r, "bearing"), 1e-6);
    EXPECT_TRUE(std::isnan(Get(r, "range")));
  }

  Send("/world/pingers/acoustics/buoy/set", "active", 0);
  ASSERT_TRUE(world.Run(0.5));
  listener.Clear();
  ASSERT_TRUE(world.Run(3.0));
  EXPECT_EQ(0u, listener.Count());
}

//////////////////////////////////////////////////
/// The world publishes every pinger, where it is and how it is set.
TEST(Acoustics, PingersGroundTruth)
{
  Readings truth("/world/pingers/acoustics/pingers");
  AcousticWorld world("pingers.sdf");
  ASSERT_TRUE(world.Run(0.5));
  const auto all = truth.All();
  ASSERT_FALSE(all.empty());
  const auto &last = all.back();
  ASSERT_EQ(1, last.children_size());
  const auto &pinger = last.children(0);
  EXPECT_EQ("buoy", pinger.params().at("name").string_value());
  EXPECT_NEAR(0.0, Get(pinger, "x"), 1e-9);
  EXPECT_NEAR(-2.0, Get(pinger, "z"), 1e-9);
  EXPECT_DOUBLE_EQ(30000.0, Get(pinger, "frequency"));
  EXPECT_DOUBLE_EQ(500.0, Get(pinger, "reach"));
  EXPECT_TRUE(GetBool(pinger, "active"));
}

//////////////////////////////////////////////////
/// The world publishes every receiver, its name, topic and band.
TEST(Acoustics, ReceiversList)
{
  Readings list("/world/pingers/acoustics/receivers");
  AcousticWorld world("pingers.sdf");
  ASSERT_TRUE(world.Run(0.5));
  const auto all = list.All();
  ASSERT_FALSE(all.empty());
  const auto &last = all.back();
  ASSERT_EQ(4, last.children_size());
  bool deaf{false};
  for (const auto &r : last.children())
  {
    const auto &p = r.params();
    if (p.at("topic").string_value() != "/deaf")
      continue;
    deaf = true;
    EXPECT_EQ("pingers::deaf::link::receiver", p.at("name").string_value());
    EXPECT_DOUBLE_EQ(40000.0, p.at("frequency").double_value());
    EXPECT_DOUBLE_EQ(1000.0, p.at("bandwidth").double_value());
  }
  EXPECT_TRUE(deaf);
}

//////////////////////////////////////////////////
/// A receiver on a model spawned while the world runs is found and hears.
TEST(Acoustics, ReceiverSpawnedAtRunTime)
{
  Readings spawned("/spawned");
  AcousticWorld world("pingers.sdf");
  ASSERT_TRUE(world.Run(0.2));

  const std::string sdf = R"(
    <sdf version="1.9" xmlns:gz="http://gazebosim.org/schema">
    <model name="spawned">
      <static>true</static>
      <pose>-30 0 -2 0 0 0</pose>
      <link name="link">
        <sensor name="receiver" type="custom" gz:type="acoustic_receiver">
          <topic>/spawned</topic>
        </sensor>
      </link>
    </model>
    </sdf>)";
  transport::Node node;
  msgs::EntityFactory req;
  req.set_sdf(sdf);
  msgs::Boolean rep;
  bool result{false};
  // The service is served by the server thread, so run it in the background.
  std::thread runner([&]() { world.Run(0.1); });
  ASSERT_TRUE(node.Request("/world/pingers/create", req, 5000, rep, result));
  runner.join();
  EXPECT_TRUE(result && rep.data());

  ASSERT_TRUE(world.Run(3.0));
  const auto readings = spawned.All();
  ASSERT_GE(readings.size(), 2u);
  // The pinger is 30 m ahead.
  EXPECT_NEAR(0.0, Get(readings.back(), "bearing"), 1e-6);
}

//////////////////////////////////////////////////
/// Two pingers on one period ping in turn, and are heard in turn; moved onto
/// the same offset they arrive together and spoil each other.
TEST(Acoustics, InTurnThenOverlapping)
{
  Readings middle("/middle");
  AcousticWorld world("two_pingers.sdf");
  ASSERT_TRUE(world.Run(3.0));
  auto readings = middle.All();
  // Pings at 0, 0.5, 1, 1.5, 2 and 2.5 s, alternately a and b.
  ASSERT_EQ(6u, readings.size());
  for (std::size_t i = 0; i < readings.size(); ++i)
  {
    EXPECT_TRUE(GetBool(readings[i], "valid"));
    EXPECT_NEAR(0.5 * static_cast<double>(i),
                Get(readings[i], "emit_time"), 1e-9);
    // a is to the south, b to the north.
    EXPECT_NEAR(i % 2 == 0 ? -GZ_PI / 2 : GZ_PI / 2,
                Get(readings[i], "bearing"), 1e-6);
  }

  Send("/world/two_pingers/acoustics/pinger_b/set", "offset", 0);
  ASSERT_TRUE(world.Run(0.5));
  middle.Clear();
  ASSERT_TRUE(world.Run(2.0));
  readings = middle.All();
  ASSERT_EQ(4u, readings.size());
  for (const auto &r : readings)
    EXPECT_FALSE(GetBool(r, "valid"));
}

//////////////////////////////////////////////////
/// Everything random comes from the world's seed: the same seed hears the
/// same pings with the same values, noise, outliers and dropouts included;
/// another seed hears others.
TEST(Acoustics, SeededRunsRepeat)
{
  // What a run hears: when each ping was sent, and its bearing.
  using Run = std::vector<std::pair<double, double>>;
  auto run = [](const std::string &_file)
  {
    Readings readings("/seeded");
    {
      AcousticWorld world(_file);
      EXPECT_TRUE(world.Run(10.0));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    Run out;
    for (const auto &r : readings.All())
      out.emplace_back(Get(r, "emit_time"), Get(r, "bearing"));
    return out;
  };

  const Run first = run("seeded.sdf");
  const Run second = run("seeded.sdf");
  ASSERT_FALSE(first.empty());
  EXPECT_EQ(first, second);

  // Half the 40 pings are lost on average at 150 m of a 200 m reach whose
  // dropout starts half way, and the bearings are noisy.
  EXPECT_GT(first.size(), 5u);
  EXPECT_LT(first.size(), 35u);
  double spread{0.0};
  for (const auto &[t, b] : first)
    spread = std::max(spread, std::abs(std::abs(b) - GZ_PI));
  EXPECT_GT(spread, 0.0);

  // Another seed, the same world otherwise.
  std::ifstream in(World("seeded.sdf"));
  std::stringstream text;
  text << in.rdbuf();
  std::string sdf = text.str();
  const auto at = sdf.find("<seed>11</seed>");
  ASSERT_NE(std::string::npos, at);
  sdf.replace(at, 15, "<seed>12</seed>");
  const std::string other = common::joinPaths(
      common::tempDirectoryPath(), "acoustics_seeded_12.sdf");
  std::ofstream(other) << sdf;
  const Run third = run(other);
  EXPECT_NE(first, third);
}

