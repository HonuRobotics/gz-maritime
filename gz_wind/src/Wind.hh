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

  /// \brief Windage on marked collisions.
  ///
  /// A world system that pushes on the shapes the wind sees. A vehicle marks
  /// those shapes with `gz:wind="true"` on a collision, the same way it marks
  /// displacement shapes for buoyancy, and the system finds them on every
  /// model, spawned later under any name included. For each marked shape it
  /// takes the projected area per shape axis from the geometry, cuts the
  /// part below the waterline, and applies quadratic drag,
  /// 0.5 * rho * Cd * A * |v| * v per axis, at the centre of the exposed part
  /// in the shape frame, so a tall shape heels and turns its link.
  ///
  /// The wind is read from the wind entity every Gazebo world carries, set by
  /// the world's `<wind><linear_velocity>` or by a system that writes it.
  /// Gazebo's `enable_wind` flag is not the mark: it belongs to the mass
  /// based force of the upstream wind effects system.
  ///
  /// World plugin parameters:
  ///
  /// * `<air_density>`: kg/m^3, default 1.225.
  /// * `<water_level>`: world z of the waterline, default 0.
  /// * `<default_drag_coefficient>`: Cd for shapes without `gz:wind_cd`,
  ///   default 1.
  ///
  /// Collision attributes:
  ///
  /// * `gz:wind="true"`: the shape is exposed to the wind.
  /// * `gz:wind_cd="1.2"`: drag coefficient for this shape.
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
