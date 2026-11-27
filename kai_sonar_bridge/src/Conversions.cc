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

#include "kai_sonar_bridge/Conversions.hh"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <marine_acoustic_msgs/msg/detection_flag.hpp>
#include <marine_acoustic_msgs/msg/sonar_image_data.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace kai_sonar_bridge
{
namespace
{
using ImageData = marine_acoustic_msgs::msg::SonarImageData;

/// \brief The least half-width a fan is drawn with, rad: a degree, so one
/// beam, or a fan narrower than a pixel, is still a visible wedge.
constexpr double kMinHalfFan = M_PI / 180.0;

//////////////////////////////////////////////////
/// \brief A ROS stamp from a gz header's.
/// \param[in] _h The gz header.
/// \return The stamp.
builtin_interfaces::msg::Time StampOf(const gz::msgs::Header &_h)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<std::int32_t>(_h.stamp().sec());
  t.nanosec = static_cast<std::uint32_t>(_h.stamp().nsec());
  return t;
}
}  // namespace

//////////////////////////////////////////////////
std::string DefaultFrame(const gz::msgs::Header &_header)
{
  for (const auto &d : _header.data())
  {
    if (d.key() == "frame_id" && d.value_size() > 0)
      return d.value(0) + "_acoustic";
  }
  return "_acoustic";
}

//////////////////////////////////////////////////
bool ImageDtype(int _type, std::uint32_t &_dtype, std::size_t &_bytes)
{
  switch (_type)
  {
    case GzImage::UINT8:
      _dtype = ImageData::DTYPE_UINT8;
      _bytes = 1;
      return true;
    case GzImage::FLOAT32:
      _dtype = ImageData::DTYPE_FLOAT32;
      _bytes = 4;
      return true;
    default:
      return false;
  }
}

//////////////////////////////////////////////////
geometry_msgs::msg::Vector3 BeamDirection(double _azimuth)
{
  geometry_msgs::msg::Vector3 d;
  d.x = 0.0;
  d.y = -std::sin(_azimuth);
  d.z = std::cos(_azimuth);
  return d;
}

//////////////////////////////////////////////////
double BeamAzimuth(const geometry_msgs::msg::Vector3 &_direction)
{
  return std::atan2(-_direction.y, _direction.z);
}

//////////////////////////////////////////////////
bool ToProjectedSonarImage(const GzImage &_in, const std::string &_frame,
  RosImage &_out)
{
  const std::size_t beams = _in.beam_count();
  const std::size_t samples = _in.sample_count();
  std::uint32_t dtype{0};
  std::size_t bytes{0};
  if (!ImageDtype(_in.data_type(), dtype, bytes) || beams == 0 ||
      samples == 0 || _in.data().size() != beams * samples * bytes ||
      static_cast<std::size_t>(_in.azimuths_size()) != beams ||
      static_cast<std::size_t>(_in.ranges_size()) != samples)
  {
    return false;
  }

  _out.header.stamp = StampOf(_in.header());
  _out.header.frame_id = _frame.empty() ? DefaultFrame(_in.header()) : _frame;
  _out.ping_info.frequency = static_cast<float>(_in.frequency());
  _out.ping_info.sound_speed = static_cast<float>(_in.sound_speed());
  _out.ping_info.tx_beamwidths.assign(1, _in.beam_width_vertical());
  _out.ping_info.rx_beamwidths.assign(1, _in.beam_width_horizontal());
  _out.beam_directions.resize(beams);
  for (std::size_t j = 0; j < beams; ++j)
    _out.beam_directions[j] = BeamDirection(_in.azimuths(static_cast<int>(j)));
  _out.ranges.assign(_in.ranges().begin(), _in.ranges().end());

  auto &img = _out.image;
  img.is_bigendian = false;
  img.dtype = dtype;
  img.beam_count = static_cast<std::uint32_t>(beams);
  img.data.resize(_in.data().size());
  const auto *src = reinterpret_cast<const std::uint8_t *>(_in.data().data());
  // Beam-major in, range-major out.
  for (std::size_t j = 0; j < beams; ++j)
  {
    for (std::size_t k = 0; k < samples; ++k)
    {
      std::memcpy(&img.data[(k * beams + j) * bytes],
        &src[(j * samples + k) * bytes], bytes);
    }
  }
  return true;
}

//////////////////////////////////////////////////
bool ToSonarDetections(const GzDetections &_in, const std::string &_frame,
  RosDetections &_out)
{
  const auto n = _in.angles_size();
  if (_in.travel_times_size() != n || _in.intensities_size() != n ||
      _in.flags_size() != n)
  {
    return false;
  }
  _out.header.stamp = StampOf(_in.header());
  _out.header.frame_id = _frame.empty() ? DefaultFrame(_in.header()) : _frame;
  _out.ping_info.frequency = static_cast<float>(_in.frequency());
  _out.ping_info.sound_speed = static_cast<float>(_in.sound_speed());
  _out.ping_info.tx_beamwidths.assign(1, _in.beam_width_along());
  _out.ping_info.rx_beamwidths.assign(1, _in.beam_width_across());
  const auto size = static_cast<std::size_t>(n);
  _out.flags.resize(size);
  _out.two_way_travel_times.resize(size);
  _out.tx_delays.assign(size, 0.0f);
  _out.intensities.resize(size);
  _out.tx_angles.assign(size, 0.0f);
  _out.rx_angles.resize(size);
  for (int i = 0; i < n; ++i)
  {
    const auto s = static_cast<std::size_t>(i);
    _out.flags[s].flag = static_cast<std::uint8_t>(_in.flags(i));
    _out.two_way_travel_times[s] = _in.travel_times(i);
    // An angle-of-arrival profiler reports no intensity (NaN in the gz
    // message); ROS consumers get 0 there rather than a NaN.
    const float level = _in.intensities(i);
    _out.intensities[s] = std::isnan(level) ? 0.0f : level;
    _out.rx_angles[s] = -_in.angles(i);
  }
  return true;
}

//////////////////////////////////////////////////
geometry_msgs::msg::Point SoundingPoint(double _rxAngle, double _travelTime,
  double _soundSpeed)
{
  const double r = 0.5 * _soundSpeed * _travelTime;
  geometry_msgs::msg::Point p;
  p.x = 0.0;
  p.y = r * std::sin(_rxAngle);
  p.z = r * std::cos(_rxAngle);
  return p;
}

//////////////////////////////////////////////////
sensor_msgs::msg::PointCloud2 SoundingsCloud(const RosDetections &_det)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = _det.header;
  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2Fields(4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  const std::size_t n = std::min({_det.flags.size(),
    _det.two_way_travel_times.size(), _det.rx_angles.size(),
    _det.intensities.size()});
  std::size_t good = 0;
  for (std::size_t i = 0; i < n; ++i)
    good += _det.flags[i].flag == 0 ? 1 : 0;
  mod.resize(good);
  // The iterators take the address of the first byte, so a ping with no
  // good soundings (all flagged, or nothing in range) is an empty cloud
  // made without them.
  if (good == 0)
    return cloud;
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> in(cloud, "intensity");
  for (std::size_t i = 0; i < n; ++i)
  {
    if (_det.flags[i].flag != 0)
      continue;
    const auto p = SoundingPoint(_det.rx_angles[i],
      _det.two_way_travel_times[i], _det.ping_info.sound_speed);
    *x = static_cast<float>(p.x);
    *y = static_cast<float>(p.y);
    *z = static_cast<float>(p.z);
    *in = _det.intensities[i];
    ++x; ++y; ++z; ++in;
  }
  return cloud;
}

//////////////////////////////////////////////////
double FanSize(double _rMax, double _aMin, double _aMax, int _maxSide,
  int &_width, int &_height)
{
  // Symmetric about the axis, so the sonar sits at the bottom center; at
  // least a degree either side, so a one-beam fan still has a width.
  const double half = std::min(std::max({std::abs(_aMin), std::abs(_aMax),
    kMinHalfFan}), M_PI / 2.0);
  const double boxWidth = 2.0 * _rMax * std::sin(half);
  const double scale = _maxSide / std::max(boxWidth, _rMax);
  _width = std::max(1, static_cast<int>(std::lround(boxWidth * scale)));
  _height = static_cast<int>(std::ceil(_rMax * scale)) + 1;
  return scale;
}

//////////////////////////////////////////////////
FanLayout LayOutFan(const RosImage &_image, int _maxSide)
{
  FanLayout layout;
  const std::size_t beams = _image.beam_directions.size();
  const std::size_t samples = _image.ranges.size();
  if (beams == 0 || samples == 0 || _maxSide < 1)
    return layout;
  const double rLast = _image.ranges.back();
  if (!(rLast > 0.0) || !std::isfinite(rLast))
    return layout;
  // One sample has no extent in range: it is drawn from the sonar out.
  const double rFirst = samples > 1 ? _image.ranges.front() : 0.0;
  const double aFirst = BeamAzimuth(_image.beam_directions.front());
  const double aLast = BeamAzimuth(_image.beam_directions.back());
  // The ends of the fan and of the range window size the image, so a
  // message whose ends are not numbers has no view, rather than a view of a
  // garbage size. gz_sonar never sends one; another publisher might.
  if (!std::isfinite(aFirst) || !std::isfinite(aLast) ||
      !std::isfinite(rFirst))
  {
    return layout;
  }
  // A fan narrower than kMinHalfFan either side of its middle is drawn
  // that wide, the width FanSize gives its box, with its beams spread over
  // the width. One beam's first and last azimuth are the same angle, which
  // no pixel's center has, and beams closer than a pixel are skipped, so
  // without this a one-beam fan was a black image and a very narrow one
  // lost beams.
  const double aMid = 0.5 * (aFirst + aLast);
  const bool spread = std::abs(aLast - aFirst) < 2.0 * kMinHalfFan;
  const double aMin = spread ? aMid - kMinHalfFan : std::min(aFirst, aLast);
  const double aMax = spread ? aMid + kMinHalfFan : std::max(aFirst, aLast);
  const double scale = FanSize(rLast, aMin, aMax, _maxSide, layout.width,
    layout.height);
  layout.lookup.assign(
    static_cast<std::size_t>(layout.width) * layout.height, -1);
  const double dr = samples > 1 ? (rLast - rFirst) / (samples - 1) : 0.0;
  const double da = beams > 1 ? (aLast - aFirst) / (beams - 1) : 0.0;
  for (int v = 0; v < layout.height; ++v)
  {
    for (int u = 0; u < layout.width; ++u)
    {
      // Forward is up; port (positive azimuth) is to the left.
      const double fwd = (layout.height - 1 - v) / scale;
      const double left = (0.5 * layout.width - u) / scale;
      const double r = std::hypot(fwd, left);
      const double a = std::atan2(left, fwd);
      if (r < rFirst || r > rLast || a < aMin || a > aMax)
        continue;
      // The nearest beam and sample; in a spread fan, each beam has an
      // equal share of the width, the first beam at the first's side.
      double jf{0.0};
      if (spread)
      {
        const double t = (a - aMin) / (aMax - aMin);
        jf = std::floor((aLast >= aFirst ? t : 1.0 - t) * beams);
      }
      else
      {
        jf = std::round((a - aFirst) / da);
      }
      const double kf = dr > 0.0 ? std::round((r - rFirst) / dr) : 0.0;
      const auto j = static_cast<std::size_t>(
        std::clamp(jf, 0.0, static_cast<double>(beams - 1)));
      const auto k = static_cast<std::size_t>(
        std::clamp(kf, 0.0, static_cast<double>(samples - 1)));
      layout.lookup[static_cast<std::size_t>(v) * layout.width + u] =
        static_cast<int>(k * beams + j);
    }
  }
  return layout;
}

//////////////////////////////////////////////////
sensor_msgs::msg::Image DrawFan(const RosImage &_image,
  const FanLayout &_layout)
{
  sensor_msgs::msg::Image view;
  view.header = _image.header;
  view.width = static_cast<std::uint32_t>(_layout.width);
  view.height = static_cast<std::uint32_t>(_layout.height);
  view.encoding = "mono8";
  view.step = view.width;
  view.data.assign(_layout.lookup.size(), 0);
  const auto &src = _image.image.data;
  for (std::size_t p = 0; p < _layout.lookup.size(); ++p)
  {
    const int i = _layout.lookup[p];
    if (i >= 0 && static_cast<std::size_t>(i) < src.size())
      view.data[p] = src[static_cast<std::size_t>(i)];
  }
  return view;
}

//////////////////////////////////////////////////
std::vector<std::uint8_t> WaterfallLine(const RosImage &_image, double _span)
{
  const std::size_t samples = _image.ranges.size();
  const std::size_t beams = std::max<std::size_t>(_image.image.beam_count, 1);
  std::vector<std::uint8_t> line(samples, 0);
  if (_image.image.data.size() < samples * beams * sizeof(float))
    return line;
  for (std::size_t k = 0; k < samples; ++k)
  {
    float db{0.0f};
    std::memcpy(&db, &_image.image.data[k * beams * sizeof(float)],
      sizeof(float));
    const double x = (db + _span) / _span;
    line[k] = static_cast<std::uint8_t>(
      std::lround(255.0 * std::clamp(x, 0.0, 1.0)));
  }
  return line;
}
}  // namespace kai_sonar_bridge
