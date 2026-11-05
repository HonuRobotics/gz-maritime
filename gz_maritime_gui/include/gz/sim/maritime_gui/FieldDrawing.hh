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
#ifndef GZ_SIM_MARITIME_GUI_FIELDDRAWING_HH_
#define GZ_SIM_MARITIME_GUI_FIELDDRAWING_HH_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/marker.pb.h>

#include "gz/sim/maritime_gui/FieldSpec.hh"
#include "gz/sim/maritime_gui/Flow.hh"

namespace tinyxml2
{
class XMLElement;
}

namespace gz::sim::maritime_gui
{
class FieldDrawingPrivate;

/// \brief How a panel draws its field.
enum class Display
{
  /// \brief Nothing.
  kOff,

  /// \brief A grid of arrows coloured by speed.
  kArrows,

  /// \brief Flow trails coloured by speed, drifting with the field.
  kTrails
};

/// \brief A display's name in a configuration: off, arrows or trails.
/// \param[in] _name The name.
/// \param[out] _display The display, when the name is one.
/// \return True if the name is a display.
bool ParseDisplay(const std::string &_name, Display &_display);

/// \brief A display's name.
/// \param[in] _display The display.
/// \return off, arrows or trails.
std::string DisplayName(Display _display);

/// \brief A panel's drawing of one field, without Qt: arrows or flow
/// trails coloured by speed, and drawings besides the field (the vehicles,
/// the wind socks), through MarkerManager.
///
/// A panel calls Draw once it knows the field.
class FieldDrawing
{
  /// \brief Constructor.
  /// \param[in] _spec The field, with its colours, scale and trails.
  /// \param[in] _log The name the plugin logs under.
  public: FieldDrawing(const FieldSpec &_spec, const std::string &_log);

  /// \brief Destructor: takes what it drew off the scene.
  public: ~FieldDrawing();

  /// \brief Read the drawing's configuration from the plugin element:
  /// `<display>` (off, arrows or trails), `<extent>`, `<spacing>`,
  /// `<center_x>`, `<center_y>`, `<scale>`, `<max_speed>`, `<particles>`,
  /// `<trail_length>`, `<lifetime>`, `<speedup>` and `<marker_service>`.
  /// \param[in] _elem The plugin element, or null.
  public: void LoadConfig(const tinyxml2::XMLElement *_elem);

  /// \brief Draw or remove the field, as the settings and the recipe say.
  /// \param[in] _available Whether the world has the field and a model.
  /// \param[in] _generation The recipe's generation.
  /// \param[in] _timeVarying Whether the field changes on its own.
  /// \param[in] _z World z of the arrows.
  /// \param[in] _velocity The field at a point.
  /// \param[in] _simTime Simulation time.
  public: void Draw(bool _available, std::uint64_t _generation,
                    bool _timeVarying, double _z,
                    const std::function<math::Vector3d(const math::Vector3d &)>
                        &_velocity,
                    const std::chrono::steady_clock::duration &_simTime);

  /// \brief Draw something besides the field (the vehicles, the wind
  /// socks), in a namespace of its own under the field's: the markers with
  /// points added or replaced, and those drawn before and empty or missing
  /// now deleted.
  /// \param[in] _kind The drawing's name, such as `vessels`.
  /// \param[in] _markers Its markers; their namespace is set here.
  public: void DrawExtra(const std::string &_kind,
                         std::vector<msgs::Marker> _markers);

  /// \brief Take a drawing made with DrawExtra off the scene.
  /// \param[in] _kind The drawing's name.
  public: void ClearExtra(const std::string &_kind);

  /// \brief How the field is drawn.
  /// \return The display.
  public: Display CurrentDisplay() const;

  /// \brief Draw the field another way.
  /// \param[in] _display The display.
  public: void SetDisplay(Display _display);

  /// \brief The colour map.
  /// \return The colour map.
  public: const Colormap &Colors() const;

  /// \brief Set the speed the colour map's last colour stands for.
  /// \param[in] _maxSpeed m/s, positive; anything else is ignored.
  /// \return True if it changed.
  public: bool SetMaxSpeed(double _maxSpeed);

  /// \brief The grid.
  /// \return The grid.
  public: const Grid &ArrowGrid() const;

  /// \brief Set the side of the grid.
  /// \param[in] _extent m, not negative; anything else is ignored.
  /// \return True if it changed.
  public: bool SetExtent(double _extent);

  /// \brief Set the distance between two arrows.
  /// \param[in] _spacing m, positive; anything else is ignored.
  /// \return True if it changed.
  public: bool SetSpacing(double _spacing);

  /// \brief m of arrow per m/s.
  /// \return The scale.
  public: double Scale() const;

  /// \brief Redraw at the next Draw, after a setting the link does not know
  /// about changed (the wind's height).
  public: void Redraw();

  /// \brief Private data pointer.
  private: std::unique_ptr<FieldDrawingPrivate> dataPtr;
};

}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_FIELDDRAWING_HH_
