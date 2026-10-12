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
#ifndef GZ_MARITIME_GUI_SCENARIOPANEL_HH_
#define GZ_MARITIME_GUI_SCENARIOPANEL_HH_

#include <memory>

#include <QString>
#include <QStringList>

#include <gz/sim/gui/GuiSystem.hh>

namespace gz::sim::maritime_gui
{
  class ScenarioPanelPrivate;

  /// \brief Named conditions for the ocean current and the wind, one click
  /// each, and a demo that steps through them.
  ///
  /// A GUI system with a button per preset. A click ramps both fields to the
  /// preset, the same way the compasses steer them: no faster than a rate
  /// (the current 0.5 m/s and 90 degrees a second, the wind 5 m/s and 90
  /// degrees a second), in `set` messages ten times a second; the wind's
  /// gusts are set at once. The demo steps through the presets in order,
  /// staying on each for its dwell in simulation time, and starts again
  /// after the last, for a talk or a stand.
  ///
  /// Configuration, all optional:
  ///
  /// * `<preset name="..." dwell="s">` with `<current speed="" direction=""/>`
  ///   and `<wind speed="" direction="" speed_gust="" direction_gust=""/>`,
  ///   each optional: the presets, replacing the built in Calm, Tidal ebb,
  ///   Fresh breeze and Squall. The current's direction is where it sets
  ///   towards, the wind's where it comes from.
  /// * `<demo>`: start the demo when the world loads, default false.
  class ScenarioPanel : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief The presets' names, in order.
    Q_PROPERTY(QStringList presets READ Presets CONSTANT)

    /// \brief Whether the demo runs.
    Q_PROPERTY(bool demo READ DemoRunning NOTIFY StateChanged)

    /// \brief The preset applied last, -1 for none.
    Q_PROPERTY(int active READ Active NOTIFY StateChanged)

    /// \brief A short sentence on what is happening.
    Q_PROPERTY(QString status READ Status NOTIFY StateChanged)

    /// \brief Constructor.
    public: ScenarioPanel();

    /// \brief Destructor.
    public: ~ScenarioPanel() override;

    // Documentation inherited.
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited.
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) override;

    /// \brief Ramp both fields to a preset.
    /// \param[in] _index The preset.
    public: Q_INVOKABLE void ApplyPreset(int _index);

    /// \brief Start or stop the demo.
    /// \param[in] _run True to start it.
    public: Q_INVOKABLE void SetDemo(bool _run);

    /// \brief The presets' names.
    /// \return The names, in order.
    public: QStringList Presets() const;

    /// \brief Whether the demo runs.
    /// \return True if it does.
    public: bool DemoRunning() const;

    /// \brief The preset applied last.
    /// \return Its index, -1 for none.
    public: int Active() const;

    /// \brief What is happening.
    /// \return A short sentence.
    public: QString Status() const;

    /// \brief The demo, the active preset or the status changed.
    signals: void StateChanged();

    /// \brief Private data pointer.
    private: std::unique_ptr<ScenarioPanelPrivate> dataPtr;
  };
}

#endif
