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
#ifndef GZ_SIM_MARITIME_GUI_PRESETS_HH_
#define GZ_SIM_MARITIME_GUI_PRESETS_HH_

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace tinyxml2
{
class XMLElement;
}

/// \brief Named conditions for both fields, and a demo that steps through
/// them, without Qt.
namespace gz::sim::maritime_gui
{
/// \brief A field's speed and direction, in the field's own convention.
struct FieldTarget
{
  /// \brief m/s.
  double speed{0.0};

  /// \brief Degrees clockwise from north: where the current sets towards,
  /// where the wind comes from.
  double direction{0.0};
};

/// \brief The wind's gusts.
struct Gusts
{
  /// \brief Standard deviation of the speed, m/s.
  double speed{0.0};

  /// \brief Standard deviation of the direction, degrees.
  double direction{0.0};
};

/// \brief Named conditions: a current and a wind to ramp to, each optional,
/// and the wind's gusts.
struct Preset
{
  /// \brief The button's label.
  std::string name;

  /// \brief The current, if the preset sets one.
  std::optional<FieldTarget> current;

  /// \brief The wind, if the preset sets one.
  std::optional<FieldTarget> wind;

  /// \brief The wind's gusts, if the preset sets them.
  std::optional<Gusts> gusts;

  /// \brief Seconds of simulation time the demo stays on it.
  double dwell{20.0};
};

/// \brief The presets built in: Calm, Tidal ebb, Fresh breeze and Squall.
/// \return The presets, in the demo's order.
std::vector<Preset> DefaultPresets();

/// \brief The `<preset>` elements of a plugin's configuration:
///
///     <preset name="Squall" dwell="20">
///       <current speed="0.5" direction="135"/>
///       <wind speed="18" direction="300" speed_gust="5"
///             direction_gust="25"/>
///     </preset>
///
/// A preset needs a name and a current or a wind; one without is skipped
/// with a warning.
/// \param[in] _elem The plugin element, or null.
/// \return The presets, in order; empty when there are none.
std::vector<Preset> ParsePresets(const tinyxml2::XMLElement *_elem);

/// \brief The demo: it steps through the presets in order, staying on each
/// for its dwell in simulation time (so a paused world pauses it), and
/// starts again after the last.
class Demo
{
  /// \brief Start from the first preset.
  public: void Start();

  /// \brief Stop where it is.
  public: void Stop();

  /// \brief Whether it runs.
  /// \return True if it does.
  public: bool Running() const;

  /// \brief The preset to apply now, when it is time for the next one.
  /// \param[in] _presets The presets.
  /// \param[in] _simTime Simulation time.
  /// \return The preset's index, or nothing.
  public: std::optional<std::size_t> Update(
      const std::vector<Preset> &_presets,
      const std::chrono::steady_clock::duration &_simTime);

  /// \brief The preset the demo is on.
  /// \return Its index, or nothing when it does not run.
  public: std::optional<std::size_t> Current() const;

  /// \brief Whether it runs.
  private: bool running{false};

  /// \brief The preset it is on, once started.
  private: std::optional<std::size_t> index;

  /// \brief Simulation time it moved to that preset.
  private: std::chrono::steady_clock::duration since{0};
};
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_PRESETS_HH_
