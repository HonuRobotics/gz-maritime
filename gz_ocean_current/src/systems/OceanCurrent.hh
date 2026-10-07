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
#ifndef GZ_MARITIME_OCEAN_CURRENT_HH_
#define GZ_MARITIME_OCEAN_CURRENT_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
  class OceanCurrentPrivate;

  /// \brief The ocean current of a world.
  ///
  /// A world system that owns the world's ocean current as a recipe, the
  /// OceanCurrentfield component on the world entity: a current model's
  /// name and its parameters, read from the world file and constant for the
  /// run. Any system reads the current at a point from that recipe through
  /// ocean_current::OceanCurrentSampler, or in one call through
  /// ocean_current::OceanCurrentAt, whatever model is behind it, whether it
  /// was loaded with the world or spawned long after. Nothing travels over
  /// transport on the way to a consumer: the recipe is a component, written
  /// as a component change, so every consumer sees it on the same step. A
  /// new current model is one class registered under a name; this system
  /// and the consumers do not change.
  ///
  /// The system publishes the current at the world's origin as ground truth
  /// on `/world/<world>/ocean_current_info`, a gz.msgs.Twist in the world
  /// frame, which ROS can bridge.
  ///
  /// World plugin parameters:
  ///
  /// * `<model>`: ocean current model, default `standard`: a speed and a
  ///   direction, uniform, horizontal and constant.
  /// * `<speed>`: m/s, the speed of the current.
  /// * `<direction>`: degrees clockwise from true north, the direction the
  ///   current sets towards, as charts draw it: 90 sets east. This is the
  ///   opposite convention from the wind, which is given by the direction it
  ///   comes from. North is the world's, from its spherical coordinates, the
  ///   one its GPS uses; with ENU and a zero heading it is +y, so 90 sets
  ///   towards +x.
  /// * `<source>`: an external source for a model that reads one, such as
  ///   the file of a gridded current; unread by the standard model.
  /// * `<seed>`: seed of anything random in a model; 0 (default) draws a
  ///   new one each run. The standard model has nothing random.
  /// * `<publish_rate>`: Hz of simulation time for the ground truth,
  ///   default 10.
  class OceanCurrent
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemReset
  {
    /// \brief Constructor.
    public: OceanCurrent();

    /// \brief Destructor.
    public: ~OceanCurrent() override;

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
    private: std::unique_ptr<OceanCurrentPrivate> dataPtr;
  };
}

#endif
