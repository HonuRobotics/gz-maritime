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

#ifndef KAI_SONAR_BRIDGE__CONVERSIONS_HH_
#define KAI_SONAR_BRIDGE__CONVERSIONS_HH_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <marine_acoustic_msgs/msg/sonar_detections.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "gz/maritime/sonar/sonar_detections.pb.h"
#include "gz/maritime/sonar/sonar_image.pb.h"

/// The bridge's conversions, free of any node, so they can be tested alone
/// and a viewer can draw from the ROS messages without the gz side.
///
/// One frame convention for everything the bridge publishes: a sonar's
/// messages are stamped in its `_acoustic` frame, which has Z along the
/// acoustic axis and X up for a forward-looking sonar, as
/// ProjectedSonarImage defines it. For a down-looking profiler that frame
/// is north-east-down: X forward, Y starboard, Z down, the frame in which
/// SonarDetections counts rx_angles positive to starboard. The sensor's
/// frame has x along the axis and its angles positive toward +y, so the
/// acoustic frame is x' = z, y' = -y, z' = x, and a sensor angle a becomes
/// a beam along (0, -sin a, cos a): an image's azimuth a and a sounding's
/// rx_angle -a are the same direction.
namespace kai_sonar_bridge
{
/// \brief The gz sonar image.
using GzImage = gz::maritime::sonar::SonarImage;

/// \brief The ROS sonar image.
using RosImage = marine_acoustic_msgs::msg::ProjectedSonarImage;

/// \brief The gz profiler soundings.
using GzDetections = gz::maritime::sonar::SonarDetections;

/// \brief The ROS profiler soundings.
using RosDetections = marine_acoustic_msgs::msg::SonarDetections;

/// \brief The frame a sonar's messages are stamped in when none is given:
/// the gz header's frame id with "_acoustic" appended.
/// \param[in] _header The gz message's header.
/// \return The frame id.
std::string DefaultFrame(const gz::msgs::Header &_header);

/// \brief The SonarImageData dtype for a gz data type, and its size.
/// \param[in] _type The gz message's data type.
/// \param[out] _dtype SonarImageData::DTYPE_UINT8 or DTYPE_FLOAT32.
/// \param[out] _bytes Bytes per sample.
/// \return False for a type the bridge does not know.
bool ImageDtype(int _type, std::uint32_t &_dtype, std::size_t &_bytes);

/// \brief A beam's unit direction in the acoustic frame.
/// \param[in] _azimuth The sensor's azimuth, about its z axis, positive
/// toward +y (to port for a forward-looking sonar), rad.
/// \return (0, -sin a, cos a).
geometry_msgs::msg::Vector3 BeamDirection(double _azimuth);

/// \brief The sensor azimuth of a beam direction, the inverse of
/// BeamDirection: atan2(-y, z).
/// \param[in] _direction A beam direction in the acoustic frame.
/// \return The azimuth, rad.
double BeamAzimuth(const geometry_msgs::msg::Vector3 &_direction);

/// \brief Convert a gz sonar image. The gz data is beam-major and the ROS
/// data range-major (the beam index fastest), so it is transposed.
/// \param[in] _in The gz message.
/// \param[in] _frame The frame to stamp; empty for DefaultFrame.
/// \param[out] _out The ROS message.
/// \return False, leaving _out unspecified, for a malformed message.
bool ToProjectedSonarImage(const GzImage &_in, const std::string &_frame,
  RosImage &_out);

/// \brief Convert a profiler's gz soundings. rx_angles are the sensor's
/// angles negated, positive to starboard; intensities the device does not
/// report (NaN) become 0.
/// \param[in] _in The gz message.
/// \param[in] _frame The frame to stamp; empty for DefaultFrame.
/// \param[out] _out The ROS message.
/// \return False, leaving _out unspecified, for a malformed message.
bool ToSonarDetections(const GzDetections &_in, const std::string &_frame,
  RosDetections &_out);

/// \brief A sounding's position in the acoustic frame: half the two-way
/// travel time times the sound speed, along (0, sin rx, cos rx).
/// \param[in] _rxAngle The sounding's rx_angle, positive to starboard, rad.
/// \param[in] _travelTime Its two-way travel time, s.
/// \param[in] _soundSpeed The sound speed, m/s.
/// \return The point, m.
geometry_msgs::msg::Point SoundingPoint(double _rxAngle, double _travelTime,
  double _soundSpeed);

/// \brief A profiler's good soundings as a cloud (x, y, z, intensity) in
/// the message's own frame.
/// \param[in] _det The soundings.
/// \return The cloud, with the soundings' header.
sensor_msgs::msg::PointCloud2 SoundingsCloud(const RosDetections &_det);

/// \brief A fan drawn as an image: its size and, per pixel, the index into
/// the ROS image's range-major data, -1 outside the fan.
struct FanLayout
{
  /// \brief Width, pixels.
  public: int width{0};

  /// \brief Height, pixels.
  public: int height{0};

  /// \brief Per pixel, row by row, the data index, or -1.
  public: std::vector<int> lookup;
};

/// \brief The image size a fan is drawn at: the fan's bounding box, forward
/// up, scaled so its longer side is _maxSide pixels. A fan narrower than a
/// degree either side is drawn as one that wide.
/// \param[in] _rMax The farthest range, m.
/// \param[in] _aMin The smallest azimuth, rad.
/// \param[in] _aMax The largest azimuth, rad.
/// \param[in] _maxSide The longer side, pixels.
/// \param[out] _width Width, pixels.
/// \param[out] _height Height, pixels.
/// \return Pixels per meter.
double FanSize(double _rMax, double _aMin, double _aMax, int _maxSide,
  int &_width, int &_height);

/// \brief Lay out a ProjectedSonarImage's fan. The beams are taken as
/// evenly spaced in azimuth, and the samples in range.
/// \param[in] _image The image; its data is not read.
/// \param[in] _maxSide The longer side, pixels.
/// \return The layout; empty for an image with no beams or ranges.
FanLayout LayOutFan(const RosImage &_image, int _maxSide);

/// \brief Draw an 8-bit ProjectedSonarImage as its fan, forward up and
/// positive azimuth (port) to the left.
/// \param[in] _image The image, laid out as _layout.
/// \param[in] _layout Its layout, from LayOutFan.
/// \return A mono8 image with the image's header.
sensor_msgs::msg::Image DrawFan(const RosImage &_image,
  const FanLayout &_layout);

/// \brief One sidescan line for a waterfall: the first beam's float32 dB
/// levels, _span dB below full scale to full scale, as 0 to 255.
/// \param[in] _image A float32 ProjectedSonarImage.
/// \param[in] _span The viewing range, dB.
/// \return The line, one value per sample.
std::vector<std::uint8_t> WaterfallLine(const RosImage &_image, double _span);
}  // namespace kai_sonar_bridge

#endif  // KAI_SONAR_BRIDGE__CONVERSIONS_HH_
