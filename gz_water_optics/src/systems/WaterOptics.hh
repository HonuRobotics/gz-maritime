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
#ifndef GZ_SIM_MARITIME_SYSTEMS_WATEROPTICS_HH_
#define GZ_SIM_MARITIME_SYSTEMS_WATEROPTICS_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
  class WaterOpticsPrivate;

  /// \brief The optics of the world's water: what it does to light.
  ///
  /// A world system that owns the water's optics as a recipe, the
  /// WaterOptics component on the world entity, so every camera in the
  /// world sees the same water and any system can read it through
  /// water_optics::WaterOpticsSampler.
  ///
  /// World plugin parameters, all optional:
  ///
  /// * `<jerlov>`: Jerlov water type, I, IA, IB, II, III, 1C, 3C, 5C, 7C,
  ///   9C. Default 1C.
  /// * `<visibility>`: horizontal visibility of a black target, m. Scales
  ///   the type's murkiness, keeping its colour.
  /// * `<attenuation>`, `<backscatter>`, `<diffuse_attenuation>`: per R G B,
  ///   1/m; `<veil>`: per R G B, a fraction of the daylight. Each replaces
  ///   the value derived from the type.
  /// * `<water_level>`: world z of the still water, m, default 0.
  /// * `<publish_rate>`: ground truth rate, Hz, simulation time, default 1.
  ///
  /// Topics, on the world's name:
  ///
  /// * `/world/<world>/water_optics/set` (gz.msgs.Param): change it while
  ///   running. Keys as the tags; a type and the vectors as strings
  ///   ("3C", "0.4 0.1 0.08"), the numbers as numbers or strings. An empty
  ///   string removes a visibility or an explicit coefficient. A message
  ///   applies whole or not at all.
  /// * `/world/<world>/water_optics_info` (gz.msgs.Param): the resolved
  ///   optics, as ground truth.
  class WaterOptics
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate,
      public ISystemReset
  {
    /// \brief Constructor.
    public: WaterOptics();

    /// \brief Destructor.
    public: ~WaterOptics() override;

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

    /// \brief Private data.
    private: std::unique_ptr<WaterOpticsPrivate> dataPtr;
  };
}  // namespace gz::sim::maritime

#endif  // GZ_SIM_MARITIME_SYSTEMS_WATEROPTICS_HH_
