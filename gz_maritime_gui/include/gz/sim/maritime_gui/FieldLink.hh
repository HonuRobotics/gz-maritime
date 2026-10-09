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

#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/param.pb.h>
#include <gz/sim/EntityComponentManager.hh>

#include "gz/sim/maritime_gui/Fields.hh"

namespace tinyxml2
{
class XMLElement;
}

namespace gz::sim::maritime_gui
{
class FieldLinkPrivate;

/// \brief What a field's panel shares with the other panels, without Qt:
/// the world's name, the field's `set` topic, a copy of the world from the
/// state service while the GUI's ECM has no recipe, what became of the last
/// change sent, and the field's arrows.
///
/// A panel calls Update at the start of its GUI update, reads its recipe
/// from Source, and calls Draw once it knows the field.
class FieldLink
{
  /// \brief Constructor.
  /// \param[in] _field The field's name on its topics: `wind` or
  /// `ocean_current`.
  /// \param[in] _log The name the plugin logs under.
  /// \param[in] _color The arrows' colour.
  /// \param[in] _scale m of arrow per m/s, by default.
  public: FieldLink(const std::string &_field, const std::string &_log,
                    const math::Color &_color, double _scale);

  /// \brief Destructor: removes the arrows it drew.
  public: ~FieldLink();

  /// \brief Read the arrows' configuration from the plugin element:
  /// `<show_arrows>`, `<extent>`, `<spacing>`, `<center_x>`, `<center_y>`,
  /// `<scale>` and `<marker_service>`.
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

  /// \brief Draw or remove the arrows, as the settings and the recipe say.
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

  /// \brief Whether the arrows are wanted.
  /// \return True if they are.
  public: bool ShowArrows() const;

  /// \brief Want the arrows or not.
  /// \param[in] _show True to draw them.
  public: void SetShowArrows(bool _show);

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
