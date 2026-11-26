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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "kai_sonar_bridge/Conversions.hh"

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

//////////////////////////////////////////////////
/// \brief The QoS of every topic the bridge publishes. Reliable, because a
/// reliable publisher serves both reliable subscribers (RViz's displays, by
/// default) and best-effort ones (the sensor-data QoS), where a best-effort
/// publisher serves only the second. Keep-last, so a slow subscriber drops
/// old pings rather than holding up the bridge.
/// \return The QoS.
rclcpp::QoS PublisherQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
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
}  // namespace

/// \brief Brings gz_sonar's sonars into ROS 2.
///
/// Parameters, as parallel lists, one entry per sonar:
///   gz_topics   the sonar's gz-transport topic
///   ros_topics  the ROS 2 topic for marine_acoustic_msgs/ProjectedSonarImage
///               or SonarDetections
///   frame_ids   the frame to stamp (optional; default: the sensor's frame
///               with "_acoustic" appended)
///   kinds       "image" (the default) for imaging and sidescan sonars, or
///               "detections" for a profiling sonar's soundings
/// and
///   view        also publish a view: <ros_topic>/view, a sensor_msgs/Image
///               of an imaging sonar's fan or a sidescan's waterfall, and
///               <ros_topic>/points, a profiler's soundings as a
///               sensor_msgs/PointCloud2 in the soundings' frame.
///
/// Images and soundings share one frame convention, the `_acoustic` frame
/// (see Conversions.hh). A ping is converted only when one of its topics
/// has a subscriber, and a view drawn only when the view has one.
class SonarBridge : public rclcpp::Node
{
  public: explicit SonarBridge(const rclcpp::NodeOptions &_options)
    : rclcpp::Node("sonar_bridge", _options)
  {
    const auto gzTopics = this->declare_parameter<std::vector<std::string>>(
      "gz_topics", std::vector<std::string>{});
    const auto rosTopics = this->declare_parameter<std::vector<std::string>>(
      "ros_topics", std::vector<std::string>{});
    const auto frameIds = this->declare_parameter<std::vector<std::string>>(
      "frame_ids", std::vector<std::string>{});
    const auto kinds = this->declare_parameter<std::vector<std::string>>(
      "kinds", std::vector<std::string>{});
    this->view = this->declare_parameter<bool>("view", true);

    if (gzTopics.size() != rosTopics.size())
    {
      RCLCPP_ERROR(this->get_logger(),
        "gz_topics and ros_topics must have the same length");
      return;
    }
    for (std::size_t i = 0; i < gzTopics.size(); ++i)
    {
      auto s = std::make_shared<Sonar>();
      s->frame = i < frameIds.size() ? frameIds[i] : "";
      if (s->frame.empty() && !gzTopics[i].empty())
      {
        // The fallback is gz's scoped sensor name, which no TF publisher
        // knows; a vehicle's generated parameters always give the frame.
        RCLCPP_WARN(this->get_logger(),
          "no frame_ids entry for %s: its messages are stamped with the gz "
          "sensor name and _acoustic, which TF does not carry", gzTopics[i].c_str());
      }
      if (i < kinds.size() && kinds[i] == "detections")
      {
        s->detPub = this->create_publisher<RosDetections>(rosTopics[i],
          PublisherQoS());
        if (this->view)
        {
          s->cloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            rosTopics[i] + "/points", PublisherQoS());
        }
        std::function<void(const GzDetections &)> dcb =
          [this, s](const GzDetections &_msg) { this->OnSoundings(*s, _msg); };
        if (!this->gzNode.Subscribe(gzTopics[i], dcb))
        {
          RCLCPP_ERROR(this->get_logger(), "cannot subscribe to [%s]",
            gzTopics[i].c_str());
          continue;
        }
        RCLCPP_INFO(this->get_logger(), "profiler [%s] -> [%s]",
          gzTopics[i].c_str(), rosTopics[i].c_str());
        this->sonars.push_back(s);
        continue;
      }
      s->pub = this->create_publisher<RosImage>(rosTopics[i],
        PublisherQoS());
      if (this->view)
      {
        s->viewPub = this->create_publisher<sensor_msgs::msg::Image>(
          rosTopics[i] + "/view", PublisherQoS());
      }
      std::function<void(const GzImage &)> cb =
        [this, s](const GzImage &_msg) { this->OnPing(*s, _msg); };
      if (!this->gzNode.Subscribe(gzTopics[i], cb))
      {
        RCLCPP_ERROR(this->get_logger(), "cannot subscribe to [%s]",
          gzTopics[i].c_str());
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "sonar [%s] -> [%s]",
        gzTopics[i].c_str(), rosTopics[i].c_str());
      this->sonars.push_back(s);
    }
  }

  private: struct Sonar
  {
    /// \brief The frame to stamp; empty for the default.
    std::string frame;
    rclcpp::Publisher<RosImage>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viewPub;
    rclcpp::Publisher<RosDetections>::SharedPtr detPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub;
    std::mutex mutex;
    /// \brief The fan's layout, kept while the geometry holds.
    FanLayout fan;
    /// \brief What the fan was laid out for: beams, ranges, first and last
    /// azimuth.
    std::vector<double> fanKey;
    /// \brief A sidescan's recent lines, newest first.
    std::vector<std::vector<std::uint8_t>> lines;
  };

  private: void OnPing(Sonar &_s, const GzImage &_m)
  {
    const bool wantData = Wanted(_s.pub);
    const bool wantView = Wanted(_s.viewPub);
    if (!wantData && !wantView)
      return;
    RosImage out;
    if (!ToProjectedSonarImage(_m, _s.frame, out))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "malformed sonar message");
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

  /// \brief An imaging sonar's frame drawn as the fan it covers, forward up.
  /// \param[in,out] _s The sonar.
  /// \param[in] _image Its frame.
  private: void DrawFan(Sonar &_s, const RosImage &_image)
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

  /// \brief A sidescan's recent lines, newest at the top, range across.
  /// \param[in,out] _s The sonar.
  /// \param[in] _image Its line.
  private: void DrawWaterfall(Sonar &_s, const RosImage &_image)
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

  /// \brief A profiler's soundings: the detections message, and the same
  /// soundings as points in its frame.
  /// \param[in,out] _s The sonar.
  /// \param[in] _m The soundings.
  private: void OnSoundings(Sonar &_s, const GzDetections &_m)
  {
    const bool wantData = Wanted(_s.detPub);
    const bool wantCloud = Wanted(_s.cloudPub);
    if (!wantData && !wantCloud)
      return;
    RosDetections out;
    if (!ToSonarDetections(_m, _s.frame, out))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "malformed detections message");
      return;
    }
    if (wantCloud)
      _s.cloudPub->publish(SoundingsCloud(out));
    if (wantData)
      _s.detPub->publish(std::move(out));
  }

  private: gz::transport::Node gzNode;
  private: bool view{true};
  private: std::vector<std::shared_ptr<Sonar>> sonars;
};
}  // namespace kai_sonar_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(kai_sonar_bridge::SonarBridge)
