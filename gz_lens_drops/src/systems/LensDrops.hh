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
#ifndef GZ_SIM_MARITIME_LENSDROPS_HH_
#define GZ_SIM_MARITIME_LENSDROPS_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz::sim::maritime
{
// Forward declaration
class LensDropsPrivate;

/// \brief Water drops on a camera's lens. Put it inside a
/// `<sensor type="camera">`:
///
/// ```xml
/// <plugin filename="gz-maritime-lens-drops-system"
///         name="gz::sim::maritime::LensDrops">
///   <rain_rate>10</rain_rate>   <!-- mm/h; default 0 -->
///   <exposure>1</exposure>      <!-- 0 sheltered to 1 open; default 1 -->
///   <focus>0.5</focus>          <!-- 0 blurred to 1 sharp; default 0.5 -->
///   <dry_time>20</dry_time>     <!-- s a drop lasts; default 20 -->
///   <seed>0</seed>              <!-- 0 draws one -->
/// </plugin>
/// ```
///
/// The drops are stepped on simulation time and drawn on the GPU by a
/// render pass on the sensor's camera (ogre2 only), so only the sensor's
/// images show them, not the GUI's view.
///
/// Topics, with `<sensor>` the sensor's scoped name, as
/// `/world/<w>/model/<m>/link/<l>/sensor/<s>`:
///
/// * `<sensor>/lens_drops/set`, `gz.msgs.Param`: `rain_rate`, `exposure`,
///   `focus`, `dry_time`, and `clear` (non-zero wipes the lens).
/// * `<sensor>/lens_drops/state`, `gz.msgs.Param`, every 0.2 s of
///   simulation time: the four parameters, `count` (drops on the lens) and
///   `attached` (true once the render pass is on the camera).
class LensDrops
    : public System,
      public ISystemConfigure,
      public ISystemPostUpdate,
      public ISystemReset
{
  /// \brief Constructor.
  public: LensDrops();

  /// \brief Destructor.
  public: ~LensDrops() override;

  // Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &_eventMgr) override;

  // Documentation inherited
  public: void PostUpdate(const UpdateInfo &_info,
                          const EntityComponentManager &_ecm) override;

  // Documentation inherited
  public: void Reset(const UpdateInfo &_info,
                     EntityComponentManager &_ecm) override;

  /// \brief Private data.
  private: std::unique_ptr<LensDropsPrivate> dataPtr;
};
}  // namespace gz::sim::maritime

#endif  // GZ_SIM_MARITIME_LENSDROPS_HH_
