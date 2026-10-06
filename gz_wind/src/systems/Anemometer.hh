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
#ifndef GZ_MARITIME_ANEMOMETER_HH_
#define GZ_MARITIME_ANEMOMETER_HH_

#include <array>
#include <chrono>

#include <gz/math/Vector3.hh>
#include <gz/sensors/Noise.hh>
#include <gz/sensors/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/Sensor.hh>

namespace gz::sim::maritime
{
  /// \brief An anemometer: a gz-sensors custom sensor,
  /// `<sensor type="custom" gz:type="anemometer">`, that publishes the
  /// apparent wind at its rate as the linear part of a gz.msgs.Twist.
  ///
  /// The wind system works out the apparent wind and hands it over with
  /// SetApparentWind; the sensor base class does the rest: topic, frame id,
  /// update rate and header sequence. An optional
  /// `<gz:anemometer><noise>` block, the same `<noise>` every Gazebo sensor
  /// takes, is applied to each axis independently.
  class Anemometer : public sensors::Sensor
  {
    // Documentation inherited.
    public: bool Load(const sdf::Sensor &_sdf) override;

    /// \brief Publish, if due, through the base class's rate check.
    public: using sensors::Sensor::Update;

    // Documentation inherited.
    public: bool Update(const std::chrono::steady_clock::duration &_now)
        override;

    // Documentation inherited.
    public: bool HasConnections() const override;

    /// \brief Set the apparent wind the next reading reports.
    /// \param[in] _wind The air's velocity relative to the sensor, in the
    /// sensor frame, m/s.
    public: void SetApparentWind(const math::Vector3d &_wind);

    /// \brief The apparent wind of the next reading, before noise.
    private: math::Vector3d apparent;

    /// \brief Noise per axis, x, y and z; null without noise.
    private: std::array<sensors::NoisePtr, 3> noise;

    /// \brief Transport node.
    private: transport::Node node;

    /// \brief Publisher of the readings.
    private: transport::Node::Publisher pub;
  };
}

#endif
