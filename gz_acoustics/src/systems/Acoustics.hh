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
#ifndef GZ_MARITIME_ACOUSTICS_HH_
#define GZ_MARITIME_ACOUSTICS_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
  class AcousticsPrivate;

  /// \brief The world's underwater acoustics, as behaviour: pingers ping,
  /// receivers hear them. No propagation model, only geometry, a delay and a
  /// few rules.
  ///
  /// A world system. It finds the pingers (the AcousticPinger component the
  /// `gz-maritime-acoustic-pinger-system` model plugin writes on a link) and
  /// the receivers (`<sensor type="custom" gz:type="acoustic_receiver">` on
  /// any link) on every model, spawned later under any name included; the
  /// world names nobody.
  ///
  /// Each active pinger pings at its offset and then every period, on
  /// simulation time, from where it is at that moment. The ping spreads at
  /// the speed of sound and reaches each receiver after the distance over
  /// that speed, at the receiver's position then. A receiver hears it when:
  ///
  /// * both ends are below the water level,
  /// * the receiver is within the pinger's reach,
  /// * the frequency is inside the receiver's band,
  /// * and it is not lost to the receiver's dropout near the reach.
  ///
  /// It then reports the direction the ping came from in its own frame,
  /// with its noise and outliers. Two pings in one receiver's band that
  /// arrive closer than the pulse length are both reported invalid.
  ///
  /// It publishes every pinger's state, as ground truth, on
  /// `/world/<world>/acoustics/pingers`: a gz.msgs.Param with one child per
  /// pinger, keys `name`, `x`, `y`, `z` (world frame, m), `frequency`,
  /// `period`, `offset`, `reach`, `pulse_length` and `active`.
  ///
  /// At the same rate it publishes the receivers on
  /// `/world/<world>/acoustics/receivers`, one child per receiver, keys
  /// `name` (scoped, from the world), `topic`, `frame_id`, `frequency`,
  /// `bandwidth` and `dropout_start`, so a
  /// GUI knows which custom sensors are receivers and where they publish.
  ///
  /// World plugin parameters:
  ///
  /// * `<speed_of_sound>`: m/s, default 1500; 0 delivers every ping at once.
  /// * `<water_level>`: world z of the water, default 0.
  /// * `<underwater_only>`: whether both ends must be below the water,
  ///   default true.
  /// * `<seed>`: seed of the dropouts and outliers, mixed with each
  ///   receiver's name; 0 (default) draws a new one each run. The noise
  ///   blocks draw from gz-math's generator, which this seed also sets when
  ///   it is not 0.
  /// * `<publish_rate>`: Hz of simulation time for the ground truth,
  ///   default 1.
  class Acoustics
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate,
      public ISystemReset
  {
    /// \brief Constructor.
    public: Acoustics();

    /// \brief Destructor.
    public: ~Acoustics() override;

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override;

    // Documentation inherited.
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<AcousticsPrivate> dataPtr;
  };
}

#endif
