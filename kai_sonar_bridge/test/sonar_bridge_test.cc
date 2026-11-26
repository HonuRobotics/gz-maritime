// Copyright 2026 Honu Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// The bridge node, in process: sonar messages published on gz-transport,
// the node spun, and what it publishes on ROS 2 checked.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gz/transport/Node.hh>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kai_sonar_bridge/Conversions.hh"
#include "kai_sonar_bridge/SonarBridge.hh"

using kai_sonar_bridge::GzDetections;
using kai_sonar_bridge::GzImage;
using kai_sonar_bridge::RosDetections;
using kai_sonar_bridge::RosImage;
using kai_sonar_bridge::SonarBridge;
using sensor_msgs::msg::Image;
using sensor_msgs::msg::PointCloud2;
using namespace std::chrono_literals;

namespace
{
/// \brief How long a wait may take before the test fails. Generous: a wait
/// ends as soon as its condition holds, so this is only spent on a failure.
constexpr auto kLimit = 20s;

//////////////////////////////////////////////////
/// \brief Wait for a condition, polling.
/// \param[in] _condition The condition.
/// \param[in] _limit The longest to wait.
/// \return Whether it came true.
bool WaitFor(const std::function<bool()> &_condition,
  std::chrono::milliseconds _limit = kLimit)
{
  const auto end = std::chrono::steady_clock::now() + _limit;
  while (std::chrono::steady_clock::now() < end)
  {
    if (_condition())
      return true;
    std::this_thread::sleep_for(5ms);
  }
  return _condition();
}

//////////////////////////////////////////////////
/// \brief A uint8 gz image whose sample k of beam j is 10 j + k + 1.
/// \param[in] _beams Beams, over +-0.5 rad, the first to port.
/// \param[in] _samples Samples, 1 m to 15 m.
/// \return The image, in the frame "probe".
GzImage ImageOf(std::size_t _beams, std::size_t _samples)
{
  GzImage m;
  m.mutable_header()->mutable_stamp()->set_sec(12);
  auto *d = m.mutable_header()->add_data();
  d->set_key("frame_id");
  d->add_value("probe");
  m.set_ping(7);
  m.set_frequency(400e3);
  m.set_sound_speed(1500.0);
  m.set_beam_count(static_cast<std::uint32_t>(_beams));
  m.set_sample_count(static_cast<std::uint32_t>(_samples));
  for (std::size_t j = 0; j < _beams; ++j)
  {
    m.add_azimuths(_beams > 1 ?
      static_cast<float>(0.5 - 1.0 * j / (_beams - 1)) : 0.0f);
  }
  for (std::size_t k = 0; k < _samples; ++k)
  {
    m.add_ranges(_samples > 1 ?
      static_cast<float>(1.0 + 14.0 * k / (_samples - 1)) : 15.0f);
  }
  m.set_data_type(GzImage::UINT8);
  std::string data;
  for (std::size_t j = 0; j < _beams; ++j)
  {
    for (std::size_t k = 0; k < _samples; ++k)
      data.push_back(static_cast<char>((10 * j + k) % 250 + 1));
  }
  m.set_data(data);
  return m;
}

//////////////////////////////////////////////////
/// \brief Three soundings, the middle one straight down at 10 m.
/// \return The soundings, in the frame "probe".
GzDetections Soundings()
{
  GzDetections m;
  m.mutable_header()->mutable_stamp()->set_sec(12);
  auto *d = m.mutable_header()->add_data();
  d->set_key("frame_id");
  d->add_value("probe");
  m.set_ping(7);
  m.set_frequency(450e3);
  m.set_sound_speed(1500.0);
  for (float a : {0.3f, 0.0f, -0.3f})
  {
    m.add_angles(a);
    m.add_travel_times(2.0f * 10.0f / 1500.0f);
    m.add_intensities(0.5f);
    m.add_flags(0u);
  }
  return m;
}

//////////////////////////////////////////////////
/// \brief A ROS subscriber that counts and keeps the last message.
template <typename T>
class Listener
{
  /// \brief Subscribe.
  /// \param[in] _node The node to subscribe with.
  /// \param[in] _topic The topic.
  /// \param[in] _qos The QoS.
  public: Listener(rclcpp::Node &_node, const std::string &_topic,
    const rclcpp::QoS &_qos = rclcpp::QoS(5))
  {
    this->sub = _node.create_subscription<T>(_topic, _qos,
      [this](std::shared_ptr<const T> _msg)
      {
        std::lock_guard<std::mutex> lock(this->mutex);
        this->last = _msg;
        ++this->count;
      });
  }

  /// \brief Messages received so far.
  /// \return The count.
  public: std::size_t Count()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->count;
  }

  /// \brief The last message.
  /// \return The message; null before the first.
  public: std::shared_ptr<const T> Last()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->last;
  }

  private: std::mutex mutex;
  private: std::shared_ptr<const T> last;
  private: std::size_t count{0};
  private: typename rclcpp::Subscription<T>::SharedPtr sub;
};

//////////////////////////////////////////////////
/// \brief The fixture: topics of this test alone, a gz node to publish the
/// sonar's messages, a ROS node to listen with, and an executor spinning
/// that node and the bridges in a thread.
class SonarBridgeTest : public ::testing::Test
{
  protected: void SetUp() override
  {
    static int serial{0};
    const std::string base = "/bridge_test_" + std::to_string(::getpid()) +
      "_" + std::to_string(++serial);
    this->gzImageTopic = base + "/gz/image";
    this->gzSoundingsTopic = base + "/gz/soundings";
    this->rosImageTopic = base + "/image";
    this->rosSoundingsTopic = base + "/soundings";
    this->imagePub = this->gzNode.Advertise<GzImage>(this->gzImageTopic);
    this->soundingsPub =
      this->gzNode.Advertise<GzDetections>(this->gzSoundingsTopic);
    ASSERT_TRUE(this->imagePub);
    ASSERT_TRUE(this->soundingsPub);
    this->listener = std::make_shared<rclcpp::Node>("listener");
    this->executor.add_node(this->listener);
    this->spinner = std::thread([this]() { this->executor.spin(); });
  }

  protected: void TearDown() override
  {
    this->executor.cancel();
    if (this->spinner.joinable())
      this->spinner.join();
  }

  /// \brief The bridge's options for this test's two sonars: the image
  /// and the soundings, both stamped "frame".
  /// \param[in] _extra More parameters; one given twice takes the later
  /// value.
  /// \return The options.
  protected: rclcpp::NodeOptions Options(
    const std::vector<rclcpp::Parameter> &_extra = {}) const
  {
    std::vector<rclcpp::Parameter> p{
      {"gz_topics", std::vector<std::string>{
        this->gzImageTopic, this->gzSoundingsTopic}},
      {"ros_topics", std::vector<std::string>{
        this->rosImageTopic, this->rosSoundingsTopic}},
      {"frame_ids", std::vector<std::string>{"frame", "frame"}},
      {"kinds", std::vector<std::string>{"image", "detections"}},
      // A short period for the lazy check, so the tests do not wait on it.
      {"subscription_heartbeat", 10}};
    for (const auto &e : _extra)
    {
      p.erase(std::remove_if(p.begin(), p.end(),
        [&e](const rclcpp::Parameter &_p)
        {
          return _p.get_name() == e.get_name();
        }), p.end());
      p.push_back(e);
    }
    return rclcpp::NodeOptions().parameter_overrides(p);
  }

  /// \brief Start a bridge and spin it.
  /// \param[in] _options Its options.
  /// \return The bridge.
  protected: std::shared_ptr<SonarBridge> Start(
    const rclcpp::NodeOptions &_options)
  {
    auto bridge = std::make_shared<SonarBridge>(_options);
    this->executor.add_node(bridge);
    return bridge;
  }

  /// \brief Stop spinning a bridge and destroy it.
  /// \param[in,out] _bridge The bridge; null afterwards.
  protected: void Stop(std::shared_ptr<SonarBridge> &_bridge)
  {
    this->executor.remove_node(_bridge);
    EXPECT_EQ(_bridge.use_count(), 1);
    _bridge.reset();
  }

  /// \brief Whether a gz publisher keeps its state for a while: for
  /// checking that a connection does not appear, or does not go.
  /// \param[in] _pub The publisher.
  /// \param[in] _connected The state it should keep.
  /// \return True if it had that state throughout, 50 lazy checks long.
  protected: static bool Stays(const gz::transport::Node::Publisher &_pub,
    bool _connected)
  {
    return !WaitFor([&]() { return _pub.HasConnections() != _connected; },
      500ms);
  }

  /// \brief Publish on gz until a condition holds.
  /// \param[in] _publish What to publish each time.
  /// \param[in] _condition The condition.
  /// \return Whether it came true within the limit.
  protected: bool PublishUntil(const std::function<void()> &_publish,
    const std::function<bool()> &_condition)
  {
    return WaitFor([&]()
      {
        _publish();
        return _condition();
      });
  }

  protected: std::string gzImageTopic;
  protected: std::string gzSoundingsTopic;
  protected: std::string rosImageTopic;
  protected: std::string rosSoundingsTopic;
  protected: gz::transport::Node gzNode;
  protected: gz::transport::Node::Publisher imagePub;
  protected: gz::transport::Node::Publisher soundingsPub;
  protected: rclcpp::Node::SharedPtr listener;
  protected: rclcpp::executors::SingleThreadedExecutor executor;
  protected: std::thread spinner;
};
}  // namespace

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, ConvertsImagesAndSoundings)
{
  auto bridge = this->Start(this->Options());
  Listener<RosImage> image(*this->listener, this->rosImageTopic);
  Listener<Image> view(*this->listener, this->rosImageTopic + "/view");
  Listener<RosDetections> soundings(*this->listener,
    this->rosSoundingsTopic);
  Listener<PointCloud2> points(*this->listener,
    this->rosSoundingsTopic + "/points");

  const GzImage gzImage = ImageOf(5, 4);
  ASSERT_TRUE(this->PublishUntil(
    [&]() { this->imagePub.Publish(gzImage); },
    [&]() { return image.Count() > 0 && view.Count() > 0; }));
  const auto img = image.Last();
  EXPECT_EQ(img->header.frame_id, "frame");
  EXPECT_EQ(img->header.stamp.sec, 12);
  EXPECT_EQ(img->image.beam_count, 5u);
  EXPECT_EQ(img->ranges.size(), 4u);
  EXPECT_EQ(img->image.dtype,
    marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8);
  ASSERT_EQ(img->image.data.size(), 20u);
  // Range-major: sample 1 of beam 3 is at 1 * 5 + 3, and holds 10 * 3 + 1 + 1.
  EXPECT_EQ(img->image.data[8], 32);
  const auto fan = view.Last();
  EXPECT_EQ(fan->header.frame_id, "frame");
  EXPECT_EQ(fan->encoding, "mono8");
  EXPECT_EQ(fan->height, 601u);
  EXPECT_EQ(fan->data.size(), static_cast<std::size_t>(fan->width) * 601u);
  // Straight ahead near the farthest range, a few rows down from the top,
  // is the middle beam's last sample.
  EXPECT_EQ(fan->data[5 * fan->width + fan->width / 2], 10 * 2 + 3 + 1);

  const GzDetections gzSoundings = Soundings();
  ASSERT_TRUE(this->PublishUntil(
    [&]() { this->soundingsPub.Publish(gzSoundings); },
    [&]() { return soundings.Count() > 0 && points.Count() > 0; }));
  const auto det = soundings.Last();
  EXPECT_EQ(det->header.frame_id, "frame");
  ASSERT_EQ(det->rx_angles.size(), 3u);
  // The sensor's angle negated: positive to starboard.
  EXPECT_FLOAT_EQ(det->rx_angles[0], -0.3f);
  EXPECT_EQ(points.Last()->header.frame_id, "frame");
  EXPECT_EQ(points.Last()->width, 3u);

  this->Stop(bridge);
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, OneBeamViewIsNotBlack)
{
  auto bridge = this->Start(this->Options());
  Listener<Image> view(*this->listener, this->rosImageTopic + "/view");
  const GzImage gzImage = ImageOf(1, 16);
  ASSERT_TRUE(this->PublishUntil(
    [&]() { this->imagePub.Publish(gzImage); },
    [&]() { return view.Count() > 0; }));
  const auto fan = view.Last();
  std::size_t lit{0};
  for (const auto p : fan->data)
    lit += p != 0 ? 1u : 0u;
  EXPECT_GT(lit, 1000u);
  this->Stop(bridge);
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, MismatchedListsBridgeNothing)
{
  auto bridge = this->Start(rclcpp::NodeOptions().parameter_overrides({
    {"gz_topics", std::vector<std::string>{
      this->gzImageTopic, this->gzSoundingsTopic}},
    {"ros_topics", std::vector<std::string>{this->rosImageTopic}}}));
  EXPECT_EQ(bridge->count_publishers(this->rosImageTopic), 0u);
  this->Stop(bridge);
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, SubscribesOnlyWhileRosListens)
{
  // HasConnections is what the sonar sensor asks before it pings.
  auto bridge = this->Start(this->Options());
  EXPECT_TRUE(Stays(this->imagePub, false));
  EXPECT_TRUE(Stays(this->soundingsPub, false));

  const GzImage gzImage = ImageOf(5, 4);
  const GzDetections gzSoundings = Soundings();
  // Each ROS topic of a sonar brings the gz subscription and takes it
  // away again, more than once, and leaves the other sonar alone.
  for (int round = 0; round < 3; ++round)
  {
    SCOPED_TRACE("round " + std::to_string(round));
    {
      Listener<RosImage> image(*this->listener, this->rosImageTopic);
      ASSERT_TRUE(WaitFor([&]() { return this->imagePub.HasConnections(); }));
      EXPECT_FALSE(this->soundingsPub.HasConnections());
      EXPECT_TRUE(this->PublishUntil(
        [&]() { this->imagePub.Publish(gzImage); },
        [&]() { return image.Count() > 0; }));
    }
    ASSERT_TRUE(WaitFor([&]() { return !this->imagePub.HasConnections(); }));
    {
      Listener<Image> view(*this->listener, this->rosImageTopic + "/view");
      ASSERT_TRUE(WaitFor([&]() { return this->imagePub.HasConnections(); }));
      EXPECT_TRUE(this->PublishUntil(
        [&]() { this->imagePub.Publish(gzImage); },
        [&]() { return view.Count() > 0; }));
    }
    ASSERT_TRUE(WaitFor([&]() { return !this->imagePub.HasConnections(); }));
    {
      Listener<RosDetections> soundings(*this->listener,
        this->rosSoundingsTopic);
      ASSERT_TRUE(WaitFor(
        [&]() { return this->soundingsPub.HasConnections(); }));
      EXPECT_FALSE(this->imagePub.HasConnections());
      EXPECT_TRUE(this->PublishUntil(
        [&]() { this->soundingsPub.Publish(gzSoundings); },
        [&]() { return soundings.Count() > 0; }));
    }
    ASSERT_TRUE(WaitFor(
      [&]() { return !this->soundingsPub.HasConnections(); }));
    {
      Listener<PointCloud2> points(*this->listener,
        this->rosSoundingsTopic + "/points");
      ASSERT_TRUE(WaitFor(
        [&]() { return this->soundingsPub.HasConnections(); }));
      EXPECT_TRUE(this->PublishUntil(
        [&]() { this->soundingsPub.Publish(gzSoundings); },
        [&]() { return points.Count() > 0; }));
    }
    ASSERT_TRUE(WaitFor(
      [&]() { return !this->soundingsPub.HasConnections(); }));
  }
  EXPECT_TRUE(Stays(this->imagePub, false));
  this->Stop(bridge);
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, StaysSubscribedUntilTheLastListenerLeaves)
{
  auto bridge = this->Start(this->Options());
  auto image = std::make_unique<Listener<RosImage>>(*this->listener,
    this->rosImageTopic);
  auto second = std::make_unique<Listener<RosImage>>(*this->listener,
    this->rosImageTopic);
  auto view = std::make_unique<Listener<Image>>(*this->listener,
    this->rosImageTopic + "/view");
  ASSERT_TRUE(WaitFor([&]() { return this->imagePub.HasConnections(); }));
  // Two on the data and one on the view: the subscription holds while any
  // of the three is left, whichever goes first.
  image.reset();
  EXPECT_TRUE(Stays(this->imagePub, true));
  second.reset();
  EXPECT_TRUE(Stays(this->imagePub, true));
  const GzImage gzImage = ImageOf(5, 4);
  EXPECT_TRUE(this->PublishUntil(
    [&]() { this->imagePub.Publish(gzImage); },
    [&]() { return view->Count() > 0; }));
  view.reset();
  EXPECT_TRUE(WaitFor([&]() { return !this->imagePub.HasConnections(); }));
  this->Stop(bridge);
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, NoViewNoViewTopics)
{
  // Without views a subscriber on the view's name is nobody's subscriber.
  auto bridge = this->Start(this->Options({{"view", false}}));
  Listener<Image> view(*this->listener, this->rosImageTopic + "/view");
  Listener<PointCloud2> points(*this->listener,
    this->rosSoundingsTopic + "/points");
  EXPECT_TRUE(Stays(this->imagePub, false));
  EXPECT_FALSE(this->soundingsPub.HasConnections());
  Listener<RosImage> image(*this->listener, this->rosImageTopic);
  EXPECT_TRUE(WaitFor([&]() { return this->imagePub.HasConnections(); }));
  this->Stop(bridge);
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, NotLazySubscribesAtOnce)
{
  auto bridge = this->Start(this->Options({{"lazy", false}}));
  EXPECT_TRUE(WaitFor([&]() { return this->imagePub.HasConnections(); }));
  EXPECT_TRUE(WaitFor([&]() { return this->soundingsPub.HasConnections(); }));
  // And holds on with a listener come and gone.
  {
    Listener<RosImage> image(*this->listener, this->rosImageTopic);
    const GzImage gzImage = ImageOf(5, 4);
    EXPECT_TRUE(this->PublishUntil(
      [&]() { this->imagePub.Publish(gzImage); },
      [&]() { return image.Count() > 0; }));
  }
  EXPECT_TRUE(Stays(this->imagePub, true));
  // Destroying the bridge lets go.
  this->Stop(bridge);
  EXPECT_TRUE(WaitFor([&]() { return !this->imagePub.HasConnections(); }));
  EXPECT_TRUE(WaitFor(
    [&]() { return !this->soundingsPub.HasConnections(); }));
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, DestroyedBridgeLetsGo)
{
  // A lazy bridge destroyed while subscribed leaves the gz topic too.
  Listener<RosImage> image(*this->listener, this->rosImageTopic);
  for (int round = 0; round < 3; ++round)
  {
    auto bridge = this->Start(this->Options());
    ASSERT_TRUE(WaitFor([&]() { return this->imagePub.HasConnections(); }));
    this->Stop(bridge);
    ASSERT_TRUE(WaitFor([&]() { return !this->imagePub.HasConnections(); }));
  }
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, HeartbeatMustBePositive)
{
  EXPECT_THROW(
    SonarBridge(this->Options({{"subscription_heartbeat", 0}})),
    rclcpp::exceptions::InvalidParameterValueException);
}

//////////////////////////////////////////////////
TEST_F(SonarBridgeTest, DestroyedWhilePingsArrive)
{
  // A gz publisher in this process runs the bridge's callback on its own
  // thread, so pings published flat out from a thread are callbacks in
  // flight whenever the bridge is destroyed. Before the bridge had a
  // destructor, such a callback ran on into the freed node.
  Listener<Image> view(*this->listener, this->rosImageTopic + "/view");
  Listener<PointCloud2> points(*this->listener,
    this->rosSoundingsTopic + "/points");
  std::atomic<bool> run{true};
  std::atomic<std::size_t> published{0};
  const GzImage gzImage = ImageOf(64, 128);
  const GzDetections gzSoundings = Soundings();
  std::vector<std::thread> publishers;
  for (int i = 0; i < 2; ++i)
  {
    publishers.emplace_back([&]()
      {
        while (run)
        {
          this->imagePub.Publish(gzImage);
          this->soundingsPub.Publish(gzSoundings);
          ++published;
          // A few thousand pings a second: far above any sonar's rate,
          // without starving the discovery the next bridge needs.
          std::this_thread::sleep_for(200us);
        }
      });
  }
  // Stops the publishers however the test leaves.
  struct Join
  {
    ~Join()
    {
      this->run = false;
      for (auto &t : this->threads)
        t.join();
    }
    std::atomic<bool> &run;
    std::vector<std::thread> &threads;
  } join{run, publishers};
  for (int i = 0; i < 26; ++i)
  {
    // Lazy and not, in turn.
    auto bridge = this->Start(this->Options({{"lazy", i % 2 == 0}}));
    // Until this bridge has drawn a view and a cloud: its callbacks run.
    const std::size_t views = view.Count();
    const std::size_t clouds = points.Count();
    ASSERT_TRUE(WaitFor([&]()
      {
        return view.Count() > views && points.Count() > clouds;
      })) << "bridge " << i;
    this->Stop(bridge);
  }
  // Pings still arrive with no bridge left: nothing is there to run.
  const std::size_t before = published;
  EXPECT_TRUE(WaitFor([&]() { return published > before + 100; }));
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // A partition of this process alone, so no other gz-transport traffic on
  // the machine reaches the test.
  ::setenv("GZ_PARTITION",
    ("kai_sonar_bridge_test_" + std::to_string(::getpid())).c_str(), 1);
  ::testing::InitGoogleTest(&_argc, _argv);
  rclcpp::init(_argc, _argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
