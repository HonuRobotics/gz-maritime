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
#ifndef GZ_MARITIME_ACOUSTICPINGER_HH_
#define GZ_MARITIME_ACOUSTICPINGER_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
  class AcousticPingerPrivate;

  /// \brief An acoustic pinger: a model plugin that puts a pinger on one of
  /// the model's links, so it moves with whatever carries it, a buoy, a
  /// mooring or a vehicle.
  ///
  /// It writes the AcousticPinger component on that link; the acoustics
  /// world system, `gz-maritime-acoustics-system`, makes it ping and decides
  /// who hears it. Without that system in the world the pinger is silent.
  ///
  /// Every number can change while the world runs, on the topic
  /// `/world/<world>/acoustics/<name>/set`: a gz.msgs.Param whose keys are the
  /// parameter names below (ROS reaches it through ros_gz_bridge as
  /// ros_gz_interfaces/msg/ParamVec). `active` takes a boolean or a number,
  /// non zero being on.
  ///
  /// Model plugin parameters:
  ///
  /// * `<name>`: unique in the world, default the model's name.
  /// * `<link_name>`: the link that carries it, default the canonical link.
  /// * `<pose>`: where it sits in that link's frame, default the origin.
  /// * `<frequency>`: Hz, default 30000.
  /// * `<period>`: s between pings, default 1.
  /// * `<offset>`: s into the period of the first ping, default 0. Pingers
  ///   that share a period and differ in offset ping in turn.
  /// * `<reach>`: m, the farthest a receiver hears it; 0 (default) is no
  ///   limit.
  /// * `<pulse_length>`: s, default 0.004; two pings that reach one receiver
  ///   in its band closer than this make both readings invalid. 0 turns the
  ///   overlap off.
  /// * `<active>`: default true.
  class AcousticPinger
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemReset
  {
    /// \brief Constructor.
    public: AcousticPinger();

    /// \brief Destructor.
    public: ~AcousticPinger() override;

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
    private: std::unique_ptr<AcousticPingerPrivate> dataPtr;
  };
}

#endif
