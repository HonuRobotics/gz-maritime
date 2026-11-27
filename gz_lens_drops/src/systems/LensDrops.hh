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
///   <enabled>true</enabled>            <!-- draw the drops; default true -->
///   <rain_rate>10</rain_rate>          <!-- mm/h, 0 to 1000; default 0 -->
///   <exposure>1</exposure>             <!-- 0 sheltered to 1 open; default 1 -->
///   <focus>0.5</focus>                 <!-- 0 blurred to 1 sharp; default 0.5 -->
///   <dry_time>20</dry_time>            <!-- s a drop lasts, 0.1 to 3600 -->
///   <contact_angle>0.96</contact_angle><!-- rad, 0.17 to pi/2; glass -->
///   <slide_radius>0.03</slide_radius>  <!-- image heights, 0.005 to 0.2 -->
///   <map_scale>0.5</map_scale>         <!-- map over image, 0.125 to 1 -->
///   <wind>true</wind>                  <!-- take the world's wind -->
///   <mask>false</mask>                 <!-- publish the drop mask -->
///   <seed>1</seed>                     <!-- 0 draws one, and logs it -->
/// </plugin>
/// ```
///
/// The drops are stepped on simulation time, at a fixed rate, with gravity
/// from the world, the sensor's motion and the world's wind (gz_wind). Right
/// before the sensor's camera draws a frame, a render pass on it (ogre2,
/// OpenGL) asks for them: they are drawn into a drop map, the height of
/// water per texel, only when they changed, and the pass refracts the image
/// through it. The pass is off while the lens is dry or `<enabled>` is
/// false. Only the sensor's images show the drops.
///
/// Topics, with `<sensor>` the sensor's scoped name, as
/// `/world/<w>/model/<m>/link/<l>/sensor/<s>`:
///
/// * `<sensor>/lens_drops/set`, `gz.msgs.Param`: `enabled`, `rain_rate`,
///   `exposure`, `focus`, `dry_time`, `contact_angle`, `slide_radius`,
///   `clear` (non-zero wipes the lens), and `add_x`, `add_y`, `add_radius`
///   together to put one drop on the lens (x and y in [0, 1], radius in
///   (0, 0.12] image heights).
/// * `<sensor>/lens_drops/state`, `gz.msgs.Param`, every 0.2 s of
///   simulation time: the parameters, `enabled`, `count`, `sliding`,
///   `covered` (share of the map with water), `flux`, `gravity_x`,
///   `gravity_y` (what the world does to the lens), `draws` (maps drawn so
///   far) and `attached` (true once the render pass is on the camera).
/// * `<sensor>/lens_drops/mask`, `gz.msgs.Image`, `L_INT8` at the map's
///   size, 255 where a drop is, with `<mask>true</mask>` only: once per
///   frame the camera draws, stamped as that frame, while subscribed.
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
