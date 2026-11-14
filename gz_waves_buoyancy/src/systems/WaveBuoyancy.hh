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
#ifndef GZ_MARITIME_WAVEBUOYANCY_HH_
#define GZ_MARITIME_WAVEBUOYANCY_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
  class WaveBuoyancyPrivate;

  /// \brief Buoyancy on the waves: vehicles float on the world's wave
  /// field, not on flat water.
  ///
  /// A world system, used instead of `gz-maritime-buoyancy-system` in a
  /// world whose physics should follow the waves; the two must not run
  /// together, or each marked shape is pushed twice. It reads the same marks:
  /// every collision marked `gz:buoyancy="true"` (a box, a cylinder or a
  /// sphere) on any model, spawned later under any name included; the world
  /// names nobody, and no vehicle changes.
  ///
  /// Every step, for each marked shape, it asks the wave field (the world's
  /// `Wavefield` component, advanced to the step) for the surface height and
  /// its normal above the shape's centre, takes the volume of the shape under
  /// that tangent plane, and pushes the link up with the weight of the water
  /// displaced, at the centre of that volume, and with the weight of the air
  /// displaced by the rest. A crest lifts a shape, a trough drops it, the
  /// slope tilts it. Without a wave field the water is flat at
  /// `<water_level>`, as with the flat buoyancy.
  ///
  /// It marks the world with the `WaveCoupling` component, holding the water
  /// level, so a system that damps against the water can tell the water
  /// moves with the waves.
  ///
  /// World plugin parameters:
  ///
  /// * `<water_density>`: kg/m^3, default 1025.
  /// * `<air_density>`: kg/m^3, above the water, default 1.
  /// * `<water_level>`: world z of the still water, default 0.
  class WaveBuoyancy
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate,
      public ISystemReset
  {
    /// \brief Constructor.
    public: WaveBuoyancy();

    /// \brief Destructor.
    public: ~WaveBuoyancy() override;

    // Documentation inherited.
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited.
    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm) override;

    // Documentation inherited.
    public: void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override;

    // Documentation inherited.
    public: void Reset(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) override;

    /// \brief Private data pointer.
    private: std::unique_ptr<WaveBuoyancyPrivate> dataPtr;
  };
}

#endif
