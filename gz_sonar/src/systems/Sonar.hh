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
#ifndef GZ_MARITIME_SONAR_SYSTEMS_SONAR_HH_
#define GZ_MARITIME_SONAR_SYSTEMS_SONAR_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
class SonarPrivate;

/// \brief The world's sonars: the world system that drives every
/// <sensor type="custom" gz:type="imaging_sonar"> (and sidescan_sonar) in
/// the world, including those on models spawned while it runs.
///
/// It owns the water and the scene's acoustic defaults:
///
///   <plugin filename="gz-maritime-sonar-system"
///           name="gz::sim::maritime::Sonar">
///     <temperature>29</temperature>       <!-- C -->
///     <salinity>30</salinity>             <!-- PSU -->
///     <sound_speed>1540</sound_speed>     <!-- m/s, the truth -->
///     <seed>0</seed>                      <!-- mixed with each sonar's name -->
///     <seabed>
///       <model>marina_bay</model>         <!-- one or more -->
///       <lambert>-27</lambert>
///     </seabed>
///     <default><lambert>-20</lambert></default>
///   </plugin>
///
/// On the simulation thread it finds the sonars and resolves every
/// visual's acoustic properties (visual, then model, then the seabed entry,
/// then the default, then -20 dB Lambert) into a table of label ids, one per
/// model and property set. On the rendering thread, which gz-sim's Sensors
/// system owns and which must be in the world, it draws each sonar's depth
/// and segmentation cameras when a ping is due, with the labels set and the
/// water surface, gz:acoustic_transparent visuals and the sonar's own model
/// hidden, and hands the images to the sensor. It joins the rendering
/// events only once a world has a sonar, so a world without one pays
/// nothing.
class Sonar
    : public System,
      public ISystemConfigure,
      public ISystemPreUpdate,
      public ISystemPostUpdate,
      public ISystemReset
{
  public: Sonar();
  public: ~Sonar() override;

  public: void Configure(const Entity &_entity,
              const std::shared_ptr<const sdf::Element> &_sdf,
              EntityComponentManager &_ecm,
              EventManager &_eventMgr) override;

  public: void PreUpdate(const UpdateInfo &_info,
              EntityComponentManager &_ecm) override;

  public: void PostUpdate(const UpdateInfo &_info,
              const EntityComponentManager &_ecm) override;

  public: void Reset(const UpdateInfo &_info,
              EntityComponentManager &_ecm) override;

  private: std::unique_ptr<SonarPrivate> dataPtr;
};
}  // namespace gz::sim::maritime

#endif  // GZ_MARITIME_SONAR_SYSTEMS_SONAR_HH_
