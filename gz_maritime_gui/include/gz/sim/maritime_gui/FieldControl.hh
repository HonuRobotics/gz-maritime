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
#ifndef GZ_SIM_MARITIME_GUI_FIELDCONTROL_HH_
#define GZ_SIM_MARITIME_GUI_FIELDCONTROL_HH_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <gz/msgs/param.pb.h>
#include <gz/sim/EntityComponentManager.hh>

#include "gz/sim/maritime_gui/Fields.hh"
#include "gz/sim/maritime_gui/FieldSpec.hh"
#include "gz/sim/maritime_gui/Flow.hh"

namespace gz::sim::maritime_gui
{
class FieldControlPrivate;

/// \brief A panel's control of one field, without Qt and without drawing:
/// the world's name, the field's `set` topic, a copy of the world from the
/// state service while the GUI's ECM has no recipe, what became of the last
/// change sent, and steering by hand at the field's rates.
///
/// A panel calls Update at the start of its GUI update, reads its recipe
/// from Source, then calls Track and StepSteering with it.
class FieldControl
{
  /// \brief Constructor.
  /// \param[in] _spec The field.
  /// \param[in] _log The name the plugin logs under.
  public: FieldControl(const FieldSpec &_spec, const std::string &_log);

  /// \brief Destructor.
  public: ~FieldControl();

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
  /// \param[in] _now The time now.
  public: void Send(const msgs::Param &_msg, std::uint64_t _generation,
      ChangeTracker::Clock::time_point _now = ChangeTracker::Clock::now());

  /// \brief What became of the last change sent, looking at the recipe.
  /// \param[in] _generation The recipe's generation now.
  /// \param[in] _now The time now.
  /// \return The state.
  public: ChangeTracker::State Track(std::uint64_t _generation,
      ChangeTracker::Clock::time_point _now = ChangeTracker::Clock::now());

  /// \brief Steer the field towards a speed and a direction, rate limited.
  /// \param[in] _speed m/s.
  /// \param[in] _direction Degrees, in the field's own convention.
  public: void Steer(double _speed, double _direction);

  /// \brief Send the steering's next step when one is due: an intermediate
  /// step untracked, the last one tracked like a change from Apply.
  /// \param[in] _speed The recipe's speed now.
  /// \param[in] _direction The recipe's direction now.
  /// \param[in] _generation The recipe's generation now.
  /// \param[in] _now The time now.
  public: void StepSteering(double _speed, double _direction,
      std::uint64_t _generation,
      Steering::Clock::time_point _now = Steering::Clock::now());

  /// \brief Whether steering is on its way to a target.
  /// \return True if it is.
  public: bool IsSteering() const;

  /// \brief Whether steering stopped short because someone else moved the
  /// field, until the next Steer or Send.
  /// \return True if it did.
  public: bool Yielded() const;

  /// \brief Private data pointer.
  private: std::unique_ptr<FieldControlPrivate> dataPtr;
};

/// \brief The status line for a change's state.
/// \param[in] _state The state.
/// \return A short sentence, empty when nothing was sent.
std::string StatusText(ChangeTracker::State _state);

/// \brief The status line for steering that stopped short.
/// \return A short sentence.
std::string YieldedText();

/// \brief A number typed in a panel: a decimal point or a decimal comma,
/// whatever the locale, and surrounding spaces.
/// \param[in] _text The text.
/// \param[out] _value The number, when there is one.
/// \return True if the whole text is a finite number.
bool ParseNumber(const std::string &_text, double &_value);
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_FIELDCONTROL_HH_
