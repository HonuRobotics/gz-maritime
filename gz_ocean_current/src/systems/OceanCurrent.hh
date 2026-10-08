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
  /// name and its parameters, read from the world file. Any system reads
  /// the current at a point from that recipe through
  /// ocean_current::OceanCurrentSampler, or in one call through
  /// ocean_current::OceanCurrentAt, whatever model is behind it, whether it
  /// was loaded with the world or spawned long after. Nothing travels over
  /// transport on the way to a consumer: the recipe is a component, written
  /// as a component change, so every consumer sees it on the same step. A
  /// new current model is one class registered under a name; this system
  /// and the consumers do not change.
  ///
  /// The current changes while the world runs on the topic
  /// `/world/<world>/ocean_current/set`, a gz.msgs.Param whose keys are
  /// parameter names (ROS reaches it through ros_gz_bridge as
  /// ros_gz_interfaces/msg/ParamVec): `speed`, `direction`, `water_level`
  /// and `seed` take a number, a double or an integer; any other key is a
  /// parameter the model owns and takes a string or a number. A message is
  /// queued and applied at the next PreUpdate, as a new recipe, so a change
  /// lands on one known step. It is applied whole or not at all: one key out
  /// of range, of the wrong type, or refused by the model, and the message
  /// changes nothing. It cannot change `<model>`. A reset puts the world
  /// file's current back.
  ///
  /// The model is checked once, at the first step, since a plugin may
  /// register it after this system is configured: an unknown model, or one
  /// that refuses its parameters, is one error here, and the world has no
  /// current and no ground truth.
  ///
  /// The system publishes the current at the world's origin as ground truth
  /// on `/world/<world>/ocean_current_info`, a gz.msgs.Twist in the world
  /// frame, which ROS can bridge.
  ///
  /// World plugin parameters:
  ///
  /// * `<model>`: ocean current model, default `standard`: a speed and a
  ///   direction, uniform and horizontal.
  /// * `<speed>`: m/s, the speed of the current.
  /// * `<direction>`: degrees clockwise from true north, the direction the
  ///   current sets towards, as charts draw it: 90 sets east. This is the
  ///   opposite convention from the wind, which is given by the direction it
  ///   comes from. North is the world's, from its spherical coordinates, the
  ///   one its GPS uses; with ENU and a zero heading it is +y, so 90 sets
  ///   towards +x.
  /// * `<water_level>`: world z of the water's surface, default 0, so a
  ///   model that varies with depth knows where the surface is; unread by
  ///   the standard model.
  /// * `<parameters>`: the parameters the model owns, one element each, such
  ///   as `<source>`, the file of a gridded current. Opaque to this system,
  ///   which only stores and replicates them as text; their meaning belongs
  ///   to the model, which accepts or refuses them. The standard model takes
  ///   none.
  /// * `<seed>`: seed of anything random in a model, default 1 so a run
  ///   repeats; 0 draws a new one each run. The standard model has nothing
  ///   random.
  ///
  /// Any other element is warned about and ignored, so a typo does not leave
  /// slack water in silence.
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
