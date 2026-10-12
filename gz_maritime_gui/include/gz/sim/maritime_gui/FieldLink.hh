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
#ifndef GZ_SIM_MARITIME_GUI_FIELDLINK_HH_
#define GZ_SIM_MARITIME_GUI_FIELDLINK_HH_

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/param.pb.h>
#include <gz/sim/EntityComponentManager.hh>

#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/maritime_gui/Flow.hh"

namespace tinyxml2
{
class XMLElement;
}

namespace gz::sim::maritime_gui
{
class FieldLinkPrivate;

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

/// \brief What a field's panel shares with the other panels, without Qt:
/// the world's name, the field's `set` topic, a copy of the world from the
/// state service while the GUI's ECM has no recipe, what became of the last
/// change sent, steering by hand, and the field's drawing: arrows or flow
/// trails, coloured by speed.
///
/// A panel calls Update at the start of its GUI update, reads its recipe
/// from Source, and calls Draw once it knows the field.
class FieldLink
{
  /// \brief Constructor.
  /// \param[in] _field The field's name on its topics: `wind` or
  /// `ocean_current`.
  /// \param[in] _log The name the plugin logs under.
  /// \param[in] _colormap Colour by speed, and its default maximum.
  /// \param[in] _scale m of arrow per m/s, by default.
  /// \param[in] _trails The trails' defaults; the region and the plane come
  /// from the grid and the field.
  /// \param[in] _steering How fast steering may change the field.
  public: FieldLink(const std::string &_field, const std::string &_log,
                    const Colormap &_colormap, double _scale,
                    const ParticleField::Settings &_trails,
                    const maritime_gui::Steering &_steering);

  /// \brief Destructor: removes the arrows it drew.
  public: ~FieldLink();

  /// \brief Read the drawing's configuration from the plugin element:
  /// `<display>` (off, arrows or trails), `<extent>`, `<spacing>`,
  /// `<center_x>`, `<center_y>`, `<scale>`, `<max_speed>`, `<particles>`,
  /// `<trail_length>`, `<lifetime>`, `<speedup>` and `<marker_service>`.
  /// \param[in] _elem The plugin element, or null.
  public: void LoadConfig(const tinyxml2::XMLElement *_elem);

  /// \brief Learn the world's name, advertise the `set` topic, and take a
  /// copy of the world from the state service if one came.
  /// \param[in] _ecm The GUI's ECM.
  public: void Update(const EntityComponentManager &_ecm);

  /// \brief Where to read the recipe: the GUI's ECM when it has it, the copy
  /// from the state service otherwise. Asks the state service once when
  /// the GUI's ECM has had no recipe for about a second.
  /// \param[in] _ecm The GUI's ECM.
  /// \param[in] _inGui Whether the GUI's ECM has the recipe.
  /// \return The ECM to read the recipe from.
  public: const EntityComponentManager &Source(
      const EntityComponentManager &_ecm, bool _inGui);

  /// \brief Log once where the recipe was first read from.
  /// \param[in] _what The field, for the log.
  /// \param[in] _fromGui Whether it came from the GUI's ECM.
  /// \param[in] _generation The recipe's generation.
  public: void LogSource(const std::string &_what, bool _fromGui,
                         std::uint64_t _generation) const;

  /// \brief Send a `set` message and start tracking it.
  /// \param[in] _msg The message.
  /// \param[in] _generation The recipe's generation now.
  public: void Send(const msgs::Param &_msg, std::uint64_t _generation);

  /// \brief What became of the last change sent, looking at the recipe.
  /// \param[in] _generation The recipe's generation now.
  /// \return The state.
  public: ChangeTracker::State Track(std::uint64_t _generation);

  /// \brief Steer the field towards a speed and a direction, ramping.
  /// \param[in] _speed m/s.
  /// \param[in] _direction Degrees, in the field's own convention.
  public: void Steer(double _speed, double _direction);

  /// \brief Send the steering's next step when one is due: an intermediate
  /// step untracked, the last one tracked like a change from Apply.
  /// \param[in] _speed The recipe's speed now.
  /// \param[in] _direction The recipe's direction now.
  /// \param[in] _generation The recipe's generation now.
  public: void StepSteering(double _speed, double _direction,
                            std::uint64_t _generation);

  /// \brief Whether steering is on its way to a target.
  /// \return True if it is.
  public: bool IsSteering() const;

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
  private: std::unique_ptr<FieldLinkPrivate> dataPtr;
};

/// \brief The status line for a change's state.
/// \param[in] _state The state.
/// \return A short sentence, empty when nothing was sent.
std::string StatusText(ChangeTracker::State _state);

/// \brief A number typed in a panel: a decimal point or a decimal comma,
/// whatever the locale, and surrounding spaces.
/// \param[in] _text The text.
/// \param[out] _value The number, when there is one.
/// \return True if the whole text is a finite number.
bool ParseNumber(const std::string &_text, double &_value);
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_FIELDLINK_HH_
