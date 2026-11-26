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

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "kai_sonar_bridge/Conversions.hh"
#include "kai_sonar_bridge/SonarBridge.hh"

namespace kai_sonar_bridge
{
namespace
{
/// \brief Viewing range of a sidescan waterfall, dB below full scale.
constexpr double kWaterfallSpan = 60.0;

/// \brief Lines a waterfall keeps.
constexpr std::size_t kWaterfallLines = 400;

/// \brief The longer side of a drawn fan, pixels.
constexpr int kFanSide = 600;

/// \brief Default messages a publisher keeps.
constexpr int kDepth = 5;

/// \brief Default period of the lazy subscription check, ms. The first
/// ping after a ROS subscriber appears waits this long at most, and the
/// sonar pings on this long after the last one leaves; reading a few
/// counts five times a second costs nothing next to one ping.
constexpr int kHeartbeatMs = 200;

//////////////////////////////////////////////////
/// \brief The QoS of every topic the bridge publishes. Reliable by
/// default, because a reliable publisher serves both reliable subscribers
/// (RViz's displays, by default) and best-effort ones (the sensor-data
/// QoS), where a best-effort publisher serves only the second. Keep-last,
/// so a slow subscriber drops old pings rather than holding up the bridge.
/// \param[in] _reliability "reliable" or "best_effort".
/// \param[in] _depth Messages kept, at least 1.
/// \return The QoS.
rclcpp::QoS PublisherQoS(const std::string &_reliability, int _depth)
{
  rclcpp::QoS qos{rclcpp::KeepLast(static_cast<std::size_t>(_depth))};
  if (_reliability == "best_effort")
    qos.best_effort();
  else
    qos.reliable();
  return qos;
}

//////////////////////////////////////////////////
/// \brief Whether a publisher has anyone to publish to.
/// \param[in] _pub The publisher.
/// \return True if it has a subscriber.
template <typename T>
bool Wanted(const T &_pub)
{
  return _pub && _pub->get_subscription_count() > 0;
}

/// \brief What a gz callback holds to know the node is still there.
///
/// gz-transport runs a subscriber's callback on its own thread (or, for a
/// publisher in the same process, on the publisher's), and Unsubscribe does
/// not wait for a callback that has started. So a callback enters the gate
/// before it touches the node and leaves when done, and the destructor
/// closes it: closing turns every later callback away and waits for those
/// inside. The callbacks own the gate, so it outlives the node for as long
/// as gz-transport keeps one of them. A count rather than a shared mutex,
/// so that a steady stream of pings cannot hold the destructor off.
class Gate
{
  /// \brief Enter, if the gate is open. Each true return needs a Leave.
  /// \return False if the node is going or gone.
  public: bool Enter()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->open)
      return false;
    ++this->inside;
    return true;
  }

  /// \brief Leave after a successful Enter.
  public: void Leave()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (--this->inside == 0)
      this->empty.notify_all();
  }

  /// \brief Close the gate and wait until nobody is inside.
  public: void Close()
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    this->open = false;
    this->empty.wait(lock, [this]() { return this->inside == 0; });
  }

  /// \brief Protects the members below.
  private: std::mutex mutex;

  /// \brief Signaled when the last callback leaves.
  private: std::condition_variable empty;

  /// \brief False once the node is being destroyed.
  private: bool open{true};

  /// \brief Callbacks running now.
  private: int inside{0};
};

/// \brief Leaves a gate when it goes out of scope.
class Pass
{
  /// \brief Enter a gate.
  /// \param[in] _gate The gate.
  public: explicit Pass(Gate &_gate)
    : gate(_gate), entered(_gate.Enter())
  {
  }

  public: ~Pass()
  {
    if (this->entered)
      this->gate.Leave();
  }

  /// \brief Whether the gate was open.
  /// \return True if the node may be used.
  public: explicit operator bool() const
  {
    return this->entered;
  }

  private: Gate &gate;
  private: bool entered{false};
};

/// \brief One bridged sonar.
struct Sonar
{
  /// \brief Its gz-transport topic.
  public: std::string gzTopic;

  /// \brief True for a profiler's soundings, false for an image.
  public: bool detections{false};

  /// \brief The frame to stamp; empty for the default.
  public: std::string frame;

  /// \brief The image's publisher; null for a profiler.
  public: rclcpp::Publisher<RosImage>::SharedPtr pub;

  /// \brief The view's publisher; null without a view or for a profiler.
  public: rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viewPub;

  /// \brief The soundings' publisher; null for an image.
  public: rclcpp::Publisher<RosDetections>::SharedPtr detPub;

  /// \brief The soundings' cloud; null without a view or for an image.
  public: rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub;

  /// \brief The gz subscription; not valid while there is none.
  public: gz::transport::Node::Subscriber gzSub;

  /// \brief True once a refused subscription has been logged, so a topic
  /// gz-transport keeps refusing is reported once, not at every check.
  public: bool refused{false};

  /// \brief Protects the view's state below.
  public: std::mutex mutex;

  /// \brief The fan's layout, kept while the geometry holds.
  public: FanLayout fan;

  /// \brief What the fan was laid out for: beams, ranges, first and last
  /// azimuth.
  public: std::vector<double> fanKey;

  /// \brief A sidescan's recent lines, newest first.
  public: std::vector<std::vector<std::uint8_t>> lines;
};
}  // namespace

/// \brief The node's private data.
class SonarBridge::Implementation
{
  /// \brief Subscribe to a sonar's gz topic, if not subscribed already.
  /// \param[in,out] _s The sonar.
  /// \return False if gz-transport refused.
  public: bool Subscribe(Sonar &_s);

  /// \brief Leave a sonar's gz topic, if subscribed.
  /// \param[in,out] _s The sonar.
  public: void Unsubscribe(Sonar &_s);

  /// \brief The lazy check: subscribe to each sonar whose ROS topics have
  /// a subscriber, and leave each whose topics have none.
  public: void Update();

  /// \brief Convert and publish an image, and draw its view.
  /// \param[in,out] _s The sonar.
  /// \param[in] _m Its ping.
  public: void OnPing(Sonar &_s, const GzImage &_m);

  /// \brief An imaging sonar's frame drawn as the fan it covers, forward up.
  /// \param[in,out] _s The sonar.
  /// \param[in] _image Its frame.
  public: void DrawFan(Sonar &_s, const RosImage &_image);

  /// \brief A sidescan's recent lines, newest at the top, range across.
  /// \param[in,out] _s The sonar.
  /// \param[in] _image Its line.
  public: void DrawWaterfall(Sonar &_s, const RosImage &_image);

  /// \brief A profiler's soundings: the detections message, and the same
  /// soundings as points in its frame.
  /// \param[in,out] _s The sonar.
  /// \param[in] _m The soundings.
  public: void OnSoundings(Sonar &_s, const GzDetections &_m);

  /// \brief The node, for its logger and clock.
  public: rclcpp::Node *node{nullptr};

  /// \brief The gz-transport node.
  public: gz::transport::Node gzNode;

  /// \brief Whether views are published.
  public: bool view{true};

  /// \brief The sonars.
  public: std::vector<std::shared_ptr<Sonar>> sonars;

  /// \brief Runs the lazy check; null when not lazy.
  public: rclcpp::TimerBase::SharedPtr timer;

  /// \brief Shared with every gz callback; see Gate.
  public: std::shared_ptr<Gate> gate{std::make_shared<Gate>()};
};

//////////////////////////////////////////////////
SonarBridge::SonarBridge(const rclcpp::NodeOptions &_options)
  : rclcpp::Node("sonar_bridge", _options),
    dataPtr(std::make_unique<Implementation>())
{
  this->dataPtr->node = this;
  const auto gzTopics = this->declare_parameter<std::vector<std::string>>(
    "gz_topics", std::vector<std::string>{});
  const auto rosTopics = this->declare_parameter<std::vector<std::string>>(
    "ros_topics", std::vector<std::string>{});
  const auto frameIds = this->declare_parameter<std::vector<std::string>>(
    "frame_ids", std::vector<std::string>{});
  const auto kinds = this->declare_parameter<std::vector<std::string>>(
    "kinds", std::vector<std::string>{});
  this->dataPtr->view = this->declare_parameter<bool>("view", true);
  rcl_interfaces::msg::ParameterDescriptor reliabilityDesc;
  reliabilityDesc.description = "Reliability of every publisher";
  reliabilityDesc.additional_constraints = "reliable or best_effort";
  const auto reliability = this->declare_parameter<std::string>(
    "reliability", "reliable", reliabilityDesc);
  if (reliability != "reliable" && reliability != "best_effort")
  {
    // Refused rather than replaced: a misspelled best_effort that quietly
    // became reliable would only show on the lossy link it was meant for.
    throw rclcpp::exceptions::InvalidParameterValueException(
      "reliability must be reliable or best_effort, not [" + reliability +
      "]");
  }
  rcl_interfaces::msg::ParameterDescriptor depthDesc;
  depthDesc.description = "Messages each publisher keeps (keep-last)";
  depthDesc.integer_range.resize(1);
  depthDesc.integer_range[0].from_value = 1;
  depthDesc.integer_range[0].to_value = 1000;
  depthDesc.integer_range[0].step = 1;
  const auto depth = this->declare_parameter<int>("depth", kDepth,
    depthDesc);
  const rclcpp::QoS qos = PublisherQoS(reliability,
    static_cast<int>(depth));
  const bool lazy = this->declare_parameter<bool>("lazy", true);
  rcl_interfaces::msg::ParameterDescriptor heartbeatDesc;
  heartbeatDesc.description = "Period of the lazy subscription check, ms";
  heartbeatDesc.integer_range.resize(1);
  heartbeatDesc.integer_range[0].from_value = 1;
  heartbeatDesc.integer_range[0].to_value = 60000;
  heartbeatDesc.integer_range[0].step = 1;
  const auto heartbeat = this->declare_parameter<int>(
    "subscription_heartbeat", kHeartbeatMs, heartbeatDesc);

  if (gzTopics.size() != rosTopics.size())
  {
    RCLCPP_ERROR(this->get_logger(),
      "gz_topics and ros_topics must have the same length");
    return;
  }
  for (std::size_t i = 0; i < gzTopics.size(); ++i)
  {
    auto s = std::make_shared<Sonar>();
    s->gzTopic = gzTopics[i];
    s->detections = i < kinds.size() && kinds[i] == "detections";
    s->frame = i < frameIds.size() ? frameIds[i] : "";
    if (s->frame.empty() && !gzTopics[i].empty())
    {
      // The fallback is gz's scoped sensor name, which no TF publisher
      // knows; a vehicle's generated parameters always give the frame.
      RCLCPP_WARN(this->get_logger(),
        "no frame_ids entry for %s: its messages are stamped with the gz "
        "sensor name and _acoustic, which TF does not carry",
        gzTopics[i].c_str());
    }
    if (s->detections)
    {
      s->detPub = this->create_publisher<RosDetections>(rosTopics[i], qos);
      if (this->dataPtr->view)
      {
        s->cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          rosTopics[i] + "/points", qos);
      }
    }
    else
    {
      s->pub = this->create_publisher<RosImage>(rosTopics[i], qos);
      if (this->dataPtr->view)
      {
        s->viewPub = this->create_publisher<sensor_msgs::msg::Image>(
          rosTopics[i] + "/view", qos);
      }
    }
    // Lazy, the gz topic is joined by the first check that finds a ROS
    // subscriber: a gz subscriber makes the sonar ping, whoever it is.
    if (!lazy && !this->dataPtr->Subscribe(*s))
      continue;
    RCLCPP_INFO(this->get_logger(), "%s [%s] -> [%s]%s",
      s->detections ? "profiler" : "sonar", gzTopics[i].c_str(),
      rosTopics[i].c_str(), lazy ? " (lazy)" : "");
    this->dataPtr->sonars.push_back(s);
  }
  if (lazy)
  {
    this->dataPtr->timer = this->create_wall_timer(
      std::chrono::milliseconds(heartbeat),
      [this]() { this->dataPtr->Update(); });
  }
}

//////////////////////////////////////////////////
SonarBridge::~SonarBridge()
{
  // Stop the lazy check, then close the gate: that waits for the gz
  // callbacks and the check already running, and turns away any that start
  // later. With nothing of the node's running, leave gz. Only then do the
  // publishers and the node go.
  if (this->dataPtr->timer)
    this->dataPtr->timer->cancel();
  this->dataPtr->gate->Close();
  for (auto &s : this->dataPtr->sonars)
    this->dataPtr->Unsubscribe(*s);
}

//////////////////////////////////////////////////
bool SonarBridge::Implementation::Subscribe(Sonar &_s)
{
  if (_s.gzSub.Valid())
    return true;
  // The callback owns the gate and the sonar, and reaches the node only
  // through an open gate.
  auto g = this->gate;
  Sonar *s = &_s;
  if (_s.detections)
  {
    std::function<void(const GzDetections &)> cb =
      [this, g, s](const GzDetections &_msg)
      {
        const Pass pass(*g);
        if (pass)
          this->OnSoundings(*s, _msg);
      };
    _s.gzSub = this->gzNode.CreateSubscriber(_s.gzTopic, cb);
  }
  else
  {
    std::function<void(const GzImage &)> cb =
      [this, g, s](const GzImage &_msg)
      {
        const Pass pass(*g);
        if (pass)
          this->OnPing(*s, _msg);
      };
    _s.gzSub = this->gzNode.CreateSubscriber(_s.gzTopic, cb);
  }
  if (!_s.gzSub.Valid())
  {
    if (!_s.refused)
    {
      RCLCPP_ERROR(this->node->get_logger(), "cannot subscribe to [%s]",
        _s.gzTopic.c_str());
    }
    _s.refused = true;
    return false;
  }
  _s.refused = false;
  return true;
}

//////////////////////////////////////////////////
void SonarBridge::Implementation::Unsubscribe(Sonar &_s)
{
  if (_s.gzSub.Valid())
    _s.gzSub.Unsubscribe();
  _s.gzSub = gz::transport::Node::Subscriber();
}

//////////////////////////////////////////////////
void SonarBridge::Implementation::Update()
{
  // Through the gate, like a gz callback: the executor may run this on
  // another thread than the one destroying the node.
  const Pass pass(*this->gate);
  if (!pass)
    return;
  for (auto &s : this->sonars)
  {
    if (Wanted(s->pub) || Wanted(s->viewPub) || Wanted(s->detPub) ||
        Wanted(s->cloudPub))
    {
      this->Subscribe(*s);
    }
    else
    {
      this->Unsubscribe(*s);
    }
  }
}

//////////////////////////////////////////////////
void SonarBridge::Implementation::OnPing(Sonar &_s, const GzImage &_m)
{
  const bool wantData = Wanted(_s.pub);
  const bool wantView = Wanted(_s.viewPub);
  if (!wantData && !wantView)
    return;
  RosImage out;
  if (!ToProjectedSonarImage(_m, _s.frame, out))
  {
    RCLCPP_WARN_THROTTLE(this->node->get_logger(),
      *this->node->get_clock(), 5000, "malformed sonar message");
    return;
  }
  if (wantView)
  {
    if (out.image.dtype ==
        marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8)
    {
      this->DrawFan(_s, out);
    }
    else
    {
      this->DrawWaterfall(_s, out);
    }
  }
  if (wantData)
    _s.pub->publish(std::move(out));
}

//////////////////////////////////////////////////
void SonarBridge::Implementation::DrawFan(Sonar &_s, const RosImage &_image)
{
  std::lock_guard<std::mutex> lock(_s.mutex);
  const std::vector<double> key{
    static_cast<double>(_image.beam_directions.size()),
    static_cast<double>(_image.ranges.size()),
    _image.ranges.front(), _image.ranges.back(),
    BeamAzimuth(_image.beam_directions.front()),
    BeamAzimuth(_image.beam_directions.back())};
  if (key != _s.fanKey)
  {
    _s.fan = LayOutFan(_image, kFanSide);
    _s.fanKey = key;
  }
  _s.viewPub->publish(kai_sonar_bridge::DrawFan(_image, _s.fan));
}

//////////////////////////////////////////////////
void SonarBridge::Implementation::DrawWaterfall(Sonar &_s,
  const RosImage &_image)
{
  std::lock_guard<std::mutex> lock(_s.mutex);
  auto line = WaterfallLine(_image, kWaterfallSpan);
  const std::size_t samples = line.size();
  if (!_s.lines.empty() && _s.lines.front().size() != samples)
    _s.lines.clear();
  _s.lines.insert(_s.lines.begin(), std::move(line));
  if (_s.lines.size() > kWaterfallLines)
    _s.lines.pop_back();

  sensor_msgs::msg::Image view;
  view.header = _image.header;
  view.width = static_cast<std::uint32_t>(samples);
  view.height = static_cast<std::uint32_t>(_s.lines.size());
  view.encoding = "mono8";
  view.step = view.width;
  view.data.reserve(samples * _s.lines.size());
  for (const auto &l : _s.lines)
    view.data.insert(view.data.end(), l.begin(), l.end());
  _s.viewPub->publish(std::move(view));
}

//////////////////////////////////////////////////
void SonarBridge::Implementation::OnSoundings(Sonar &_s, const GzDetections &_m)
{
  const bool wantData = Wanted(_s.detPub);
  const bool wantCloud = Wanted(_s.cloudPub);
  if (!wantData && !wantCloud)
    return;
  RosDetections out;
  if (!ToSonarDetections(_m, _s.frame, out))
  {
    RCLCPP_WARN_THROTTLE(this->node->get_logger(),
      *this->node->get_clock(), 5000, "malformed detections message");
    return;
  }
  if (wantCloud)
    _s.cloudPub->publish(SoundingsCloud(out));
  if (wantData)
    _s.detPub->publish(std::move(out));
}
}  // namespace kai_sonar_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(kai_sonar_bridge::SonarBridge)
