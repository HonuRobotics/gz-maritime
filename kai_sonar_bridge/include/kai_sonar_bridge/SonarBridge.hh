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

#ifndef KAI_SONAR_BRIDGE__SONARBRIDGE_HH_
#define KAI_SONAR_BRIDGE__SONARBRIDGE_HH_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace kai_sonar_bridge
{
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
  /// \brief Constructor.
  /// \param[in] _options The node's options.
  public: explicit SonarBridge(const rclcpp::NodeOptions &_options);

  /// \brief Destructor. Leaves every gz topic and waits for a callback
  /// that is still running before the members go.
  public: ~SonarBridge() override;

  /// \brief The node's private data.
  private: class Implementation;

  /// \brief The node's private data.
  private: std::unique_ptr<Implementation> dataPtr;
};
}  // namespace kai_sonar_bridge

#endif  // KAI_SONAR_BRIDGE__SONARBRIDGE_HH_
