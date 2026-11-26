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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/transport/Node.hh>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_detections.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "gz/maritime/sonar/sonar_detections.pb.h"
#include "gz/maritime/sonar/sonar_image.pb.h"

namespace kai_sonar_bridge
{
namespace
{
using GzImage = gz::maritime::sonar::SonarImage;
using RosImage = marine_acoustic_msgs::msg::ProjectedSonarImage;
using GzDetections = gz::maritime::sonar::SonarDetections;
using RosDetections = marine_acoustic_msgs::msg::SonarDetections;

/// \brief The frame id a gz header carries.
std::string FrameOf(const gz::msgs::Header &_h)
{
  for (const auto &d : _h.data())
  {
    if (d.key() == "frame_id" && d.value_size() > 0)
      return d.value(0);
  }
  return "";
}

/// \brief Viewing range of a sidescan waterfall, dB below full scale.
constexpr double kWaterfallSpan = 60.0;

/// \brief Lines a waterfall keeps.
constexpr std::size_t kWaterfallLines = 400;

/// \brief Width of a drawn fan, pixels.
constexpr int kFanWidth = 600;

/// \brief The QoS of every topic the bridge publishes. Reliable, because a
/// reliable publisher serves both reliable subscribers (RViz's displays, by
/// default) and best-effort ones (the sensor-data QoS), where a best-effort
/// publisher serves only the second. Keep-last, so a slow subscriber drops
/// old pings rather than holding up the bridge.
rclcpp::QoS PublisherQoS()
{
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
}
}  // namespace

/// \brief Brings gz_sonar's sonars into ROS 2.
///
/// Parameters, as parallel lists, one entry per sonar:
///   gz_topics   the sonar's gz-transport topic
///   ros_topics  the ROS 2 topic for marine_acoustic_msgs/ProjectedSonarImage
///   frame_ids   the frame to stamp (optional; default: the sensor's frame
///               with "_acoustic" appended for images, the sensor's frame
///               itself for detections)
///   kinds       "image" (the default) for imaging and sidescan sonars, or
///               "detections" for a profiling sonar's soundings
/// and
///   view        also publish a view: <ros_topic>/view, a sensor_msgs/Image
///               of an imaging sonar's fan or a sidescan's waterfall, and
///               <ros_topic>/points, a profiler's soundings as a
///               sensor_msgs/PointCloud2 in the sensor's frame.
///
/// Soundings: marine_acoustic_msgs/SonarDetections counts rx_angles
/// positive to starboard for a down-looking sonar, which is the sensor's
/// -y; the sensor's angles are positive toward +y, so they are negated.
///
/// The message's frame has Z along the fan's center and X up, so a beam at
/// azimuth a (about the sensor's z axis, positive to the left) points along
/// (0, -sin a, cos a); sonar_image_proc reads it back as atan2(-y, z) = a.
/// Its data is range-major, the beam index fastest, so the sensor's
/// beam-major data is transposed.
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
          "sensor name, which TF does not carry", gzTopics[i].c_str());
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
    std::string frame;
    rclcpp::Publisher<RosImage>::SharedPtr pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr viewPub;
    rclcpp::Publisher<RosDetections>::SharedPtr detPub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub;
    std::mutex mutex;
    /// \brief A fan's pixel to (beam, sample) lookup, -1 outside.
    std::vector<int> fanLut;
    int fanW{0};
    int fanH{0};
    std::size_t fanKey{0};
    /// \brief A sidescan's recent lines, newest first.
    std::vector<std::vector<std::uint8_t>> lines;
  };

  private: void OnPing(Sonar &_s, const GzImage &_m)
  {
    const std::size_t beams = _m.beam_count();
    const std::size_t samples = _m.sample_count();
    const bool u8 = _m.data_type() == GzImage::UINT8;
    const std::size_t bytes = u8 ? 1 : 4;
    if (beams == 0 || samples == 0 ||
        _m.data().size() != beams * samples * bytes ||
        static_cast<std::size_t>(_m.azimuths_size()) != beams ||
        static_cast<std::size_t>(_m.ranges_size()) != samples)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "malformed sonar message");
      return;
    }

    RosImage out;
    out.header.stamp.sec = static_cast<int32_t>(_m.header().stamp().sec());
    out.header.stamp.nanosec =
      static_cast<uint32_t>(_m.header().stamp().nsec());
    out.header.frame_id = !_s.frame.empty() ? _s.frame :
      FrameOf(_m.header()) + "_acoustic";
    out.ping_info.frequency = static_cast<float>(_m.frequency());
    out.ping_info.sound_speed = static_cast<float>(_m.sound_speed());
    out.ping_info.tx_beamwidths.push_back(_m.beam_width_vertical());
    out.ping_info.rx_beamwidths.push_back(_m.beam_width_horizontal());
    out.beam_directions.resize(beams);
    for (std::size_t j = 0; j < beams; ++j)
    {
      const double a = _m.azimuths(static_cast<int>(j));
      out.beam_directions[j].x = 0.0;
      out.beam_directions[j].y = -std::sin(a);
      out.beam_directions[j].z = std::cos(a);
    }
    out.ranges.assign(_m.ranges().begin(), _m.ranges().end());

    auto &img = out.image;
    img.is_bigendian = false;
    img.dtype = u8 ? marine_acoustic_msgs::msg::SonarImageData::DTYPE_UINT8 :
      marine_acoustic_msgs::msg::SonarImageData::DTYPE_FLOAT32;
    img.beam_count = static_cast<uint32_t>(beams);
    img.data.resize(_m.data().size());
    const auto *src = reinterpret_cast<const std::uint8_t *>(_m.data().data());
    // Beam-major in, range-major out.
    for (std::size_t j = 0; j < beams; ++j)
    {
      for (std::size_t k = 0; k < samples; ++k)
      {
        std::memcpy(&img.data[(k * beams + j) * bytes],
          &src[(j * samples + k) * bytes], bytes);
      }
    }
    if (_s.viewPub)
    {
      if (u8)
        this->DrawFan(_s, _m, out.header);
      else
        this->DrawWaterfall(_s, _m, out.header);
    }
    _s.pub->publish(std::move(out));
  }

  /// \brief An imaging sonar's frame drawn as the fan it covers, forward up.
  private: void DrawFan(Sonar &_s, const GzImage &_m,
    const std_msgs::msg::Header &_header)
  {
    std::lock_guard<std::mutex> lock(_s.mutex);
    const std::size_t beams = _m.beam_count();
    const std::size_t samples = _m.sample_count();
    const double rMax = _m.ranges(static_cast<int>(samples) - 1);
    const double rMin = _m.ranges(0);
    const double aMin = _m.azimuths(0);
    const double aMax = _m.azimuths(static_cast<int>(beams) - 1);
    const std::size_t key = beams * 1000003u + samples +
      static_cast<std::size_t>(rMax * 1000.0);
    if (key != _s.fanKey)
    {
      // At least a degree either side, so a one-beam or narrow fan still
      // has a width to scale by.
      const double halfWidth = rMax * std::sin(std::max({std::abs(aMin),
        std::abs(aMax), M_PI / 180.0}));
      _s.fanW = kFanWidth;
      const double scale = _s.fanW / (2.0 * halfWidth);
      _s.fanH = static_cast<int>(std::ceil(rMax * scale)) + 1;
      _s.fanLut.assign(static_cast<std::size_t>(_s.fanW * _s.fanH), -1);
      const double dr = (rMax - rMin) / std::max<double>(samples - 1, 1.0);
      const double da = (aMax - aMin) / std::max<double>(beams - 1, 1.0);
      for (int v = 0; v < _s.fanH; ++v)
      {
        for (int u = 0; u < _s.fanW; ++u)
        {
          // Forward is up; port (positive azimuth) is to the left.
          const double fwd = (_s.fanH - 1 - v) / scale;
          const double left = (0.5 * _s.fanW - u) / scale;
          const double r = std::hypot(fwd, left);
          const double a = std::atan2(left, fwd);
          if (r < rMin || r > rMax || a < aMin || a > aMax)
            continue;
          const auto j = static_cast<int>(std::lround((a - aMin) / da));
          const auto k = static_cast<int>(std::lround((r - rMin) / dr));
          _s.fanLut[static_cast<std::size_t>(v * _s.fanW + u)] =
            j * static_cast<int>(samples) + k;
        }
      }
      _s.fanKey = key;
    }
    sensor_msgs::msg::Image view;
    view.header = _header;
    view.width = static_cast<uint32_t>(_s.fanW);
    view.height = static_cast<uint32_t>(_s.fanH);
    view.encoding = "mono8";
    view.step = view.width;
    view.data.assign(_s.fanLut.size(), 0);
    const auto *src = reinterpret_cast<const std::uint8_t *>(_m.data().data());
    for (std::size_t p = 0; p < _s.fanLut.size(); ++p)
    {
      if (_s.fanLut[p] >= 0)
        view.data[p] = src[_s.fanLut[p]];
    }
    _s.viewPub->publish(std::move(view));
  }

  /// \brief A sidescan's recent lines, newest at the top, range across.
  private: void DrawWaterfall(Sonar &_s, const GzImage &_m,
    const std_msgs::msg::Header &_header)
  {
    std::lock_guard<std::mutex> lock(_s.mutex);
    const std::size_t samples = _m.sample_count();
    std::vector<float> db(samples);
    std::memcpy(db.data(), _m.data().data(), samples * sizeof(float));
    std::vector<std::uint8_t> line(samples);
    for (std::size_t k = 0; k < samples; ++k)
    {
      const double x = (db[k] + kWaterfallSpan) / kWaterfallSpan;
      line[k] = static_cast<std::uint8_t>(
        std::lround(255.0 * std::clamp(x, 0.0, 1.0)));
    }
    if (!_s.lines.empty() && _s.lines.front().size() != samples)
      _s.lines.clear();
    _s.lines.insert(_s.lines.begin(), std::move(line));
    if (_s.lines.size() > kWaterfallLines)
      _s.lines.pop_back();

    sensor_msgs::msg::Image view;
    view.header = _header;
    view.width = static_cast<uint32_t>(samples);
    view.height = static_cast<uint32_t>(_s.lines.size());
    view.encoding = "mono8";
    view.step = view.width;
    view.data.reserve(samples * _s.lines.size());
    for (const auto &l : _s.lines)
      view.data.insert(view.data.end(), l.begin(), l.end());
    _s.viewPub->publish(std::move(view));
  }

  /// \brief A profiler's soundings: the detections message, and the same
  /// soundings as points in the sensor's frame (x along the acoustic axis,
  /// the fan in its x-y plane).
  private: void OnSoundings(Sonar &_s, const GzDetections &_m)
  {
    const auto n = static_cast<std::size_t>(_m.angles_size());
    if (static_cast<std::size_t>(_m.travel_times_size()) != n ||
        static_cast<std::size_t>(_m.intensities_size()) != n ||
        static_cast<std::size_t>(_m.flags_size()) != n)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "malformed detections message");
      return;
    }
    RosDetections out;
    out.header.stamp.sec = static_cast<int32_t>(_m.header().stamp().sec());
    out.header.stamp.nanosec =
      static_cast<uint32_t>(_m.header().stamp().nsec());
    out.header.frame_id = !_s.frame.empty() ? _s.frame :
      FrameOf(_m.header());
    out.ping_info.frequency = static_cast<float>(_m.frequency());
    out.ping_info.sound_speed = static_cast<float>(_m.sound_speed());
    out.ping_info.tx_beamwidths.push_back(_m.beam_width_along());
    out.ping_info.rx_beamwidths.push_back(_m.beam_width_across());
    for (std::size_t i = 0; i < n; ++i)
    {
      marine_acoustic_msgs::msg::DetectionFlag f;
      f.flag = static_cast<uint8_t>(_m.flags(static_cast<int>(i)));
      out.flags.push_back(f);
      out.two_way_travel_times.push_back(_m.travel_times(static_cast<int>(i)));
      out.tx_delays.push_back(0.0f);
      // An angle-of-arrival profiler reports no intensity (NaN in the gz
      // message); ROS consumers get 0 there rather than a NaN.
      const float level = _m.intensities(static_cast<int>(i));
      out.intensities.push_back(std::isnan(level) ? 0.0f : level);
      out.tx_angles.push_back(0.0f);
      out.rx_angles.push_back(-_m.angles(static_cast<int>(i)));
    }
    if (_s.cloudPub)
    {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header = out.header;
      sensor_msgs::PointCloud2Modifier mod(cloud);
      mod.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
      std::size_t good = 0;
      for (std::size_t i = 0; i < n; ++i)
        good += _m.flags(static_cast<int>(i)) == 0 ? 1 : 0;
      mod.resize(good);
      // The iterators take the address of the first byte, so a ping with no
      // good soundings (all flagged, or nothing in range) publishes an empty
      // cloud without them.
      if (good > 0)
      {
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> in(cloud, "intensity");
        for (std::size_t i = 0; i < n; ++i)
        {
          const int k = static_cast<int>(i);
          if (_m.flags(k) != 0)
            continue;
          const double r = 0.5 * _m.sound_speed() * _m.travel_times(k);
          const double a = _m.angles(k);
          *x = static_cast<float>(r * std::cos(a));
          *y = static_cast<float>(r * std::sin(a));
          *z = 0.0f;
          *in = std::isnan(_m.intensities(k)) ? 0.0f : _m.intensities(k);
          ++x; ++y; ++z; ++in;
        }
      }
      _s.cloudPub->publish(std::move(cloud));
    }
    _s.detPub->publish(std::move(out));
  }

  private: gz::transport::Node gzNode;
  private: bool view{true};
  private: std::vector<std::shared_ptr<Sonar>> sonars;
};
}  // namespace kai_sonar_bridge

RCLCPP_COMPONENTS_REGISTER_NODE(kai_sonar_bridge::SonarBridge)
