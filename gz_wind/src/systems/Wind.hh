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
#ifndef GZ_MARITIME_WIND_HH_
#define GZ_MARITIME_WIND_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
  class WindPrivate;

  /// \brief The wind of a world.
  ///
  /// A world system that owns the world's wind as a recipe, the Windfield
  /// component on the world entity: a wind model's name and its parameters.
  /// Any system reads the wind at a point from that recipe through
  /// wind::WindSampler, whatever model is behind it. This system writes the
  /// recipe, from the world file and from the topic
  /// `/world/<world>/wind/set`, a gz.msgs.Param whose keys are parameter
  /// names (ROS reaches it through ros_gz_bridge as
  /// ros_gz_interfaces/msg/ParamVec). It also writes the wind at the world's
  /// origin into the wind entity every Gazebo world carries, which Gazebo's
  /// rotor and wing systems read, and publishes it as ground truth on
  /// `/world/<world>/wind_info` (gz.msgs.Wind).
  ///
  /// Without `<speed>` or `<direction>` the wind starts as the world's
  /// `<wind><linear_velocity>`, so a world that sets only that keeps it.
  ///
  /// World plugin parameters:
  ///
  /// * `<model>`: wind model, default `standard`.
  /// * `<speed>`: m/s, the mean horizontal wind speed.
  /// * `<direction>`: degrees clockwise from true north, the direction the
  ///   wind comes from, as in weather reports: 270 is a wind from the west.
  ///   North is the world's, from its spherical coordinates, the one its GPS
  ///   uses; with ENU and a zero heading it is +y, so 270 blows towards +x.
  /// * `<vertical>`: m/s, positive up.
  /// * `<speed_gust>`, `<speed_gust_time>`: standard deviation (m/s) and
  ///   correlation time (s, default 2) of the gusts on the speed.
  /// * `<direction_gust>`, `<direction_gust_time>`: standard deviation
  ///   (degrees) and correlation time (s, default 10) of the gusts on the
  ///   direction. Each gust is a sum of sinusoids with a Lorentzian
  ///   spectrum, a function of time that travels with the mean wind; zero
  ///   turns it off.
  /// * `<seed>`: seed of the gusts; 0 (default) draws a new one each run.
  /// * `<publish_rate>`: Hz of simulation time for the ground truth,
  ///   default 10.
  class Wind
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemReset
  {
    /// \brief Constructor.
    public: Wind();

    /// \brief Destructor.
    public: ~Wind() override;

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    // Documentation inherited.
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<WindPrivate> dataPtr;
  };
}

#endif
