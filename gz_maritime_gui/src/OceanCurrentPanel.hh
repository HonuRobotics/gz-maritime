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
#ifndef GZ_MARITIME_GUI_OCEANCURRENTPANEL_HH_
#define GZ_MARITIME_GUI_OCEANCURRENTPANEL_HH_

#include <memory>

#include <QString>
#include <QStringList>
#include <QVariantMap>

#include <gz/sim/gui/GuiSystem.hh>

namespace gz::sim::maritime_gui
{
  class OceanCurrentPanelPrivate;

  /// \brief Change the world's ocean current, and see it.
  ///
  /// A GUI system with a panel. It reads the ocean current from its recipe
  /// on the world, which the GUI's copy of the ECM holds, and shows its
  /// values whoever changed them. Apply sends the keys the user changed as
  /// one gz.msgs.Param on `/world/<world>/ocean_current/set`; the server
  /// applies a message whole or refuses it and logs why, and the panel sees
  /// a refusal as a recipe that did not change. The direction is the one the
  /// current sets towards, in degrees clockwise from north; a number takes a
  /// decimal point or a decimal comma, whatever the locale.
  ///
  /// The compass steers it by hand: drag the arrow to turn the flow, move
  /// the slider for its speed, and the panel steps the field there, rate
  /// limited, stopping if something else moves the field.
  ///
  /// It draws the current as a grid of blue arrows just above the water,
  /// sampled from the recipe with OceanCurrentSampler, as a LINE_LIST marker
  /// through MarkerManager, redrawn when the recipe changes.
  ///
  /// A plugin added from the menu after the world is up missed the recipe;
  /// it then asks the world's state service once.
  ///
  /// Configuration, all optional:
  ///
  /// * `<display>`: `off` (default), `arrows` or `trails`.
  /// * `<on_vehicles>`: draw on each vehicle the water past its hull (a
  ///   solid arrow 1 m above it, the current less the vehicle's velocity
  ///   over the ground), its track over the ground and a line along its
  ///   heading, default false.
  /// * `<extent>`: side of the grid, m, default 40.
  /// * `<spacing>`: between two arrows, m, default 5.
  /// * `<center_x>`, `<center_y>`: centre of the grid, default the origin.
  /// * `<scale>`: m of arrow per m/s, default 4.
  /// * `<marker_service>`: MarkerManager's service, default `/marker`.
  class OceanCurrentPanel : public gz::sim::GuiSystem
  {
    Q_OBJECT

    /// \brief The ocean current: available, model, speed, direction,
    /// waterLevel, flowHeading and parameters.
    Q_PROPERTY(QVariantMap current READ Current NOTIFY CurrentChanged)

    /// \brief What became of the last change.
    Q_PROPERTY(QString status READ Status NOTIFY StatusChanged)

    /// \brief How the field is drawn: off, arrows or trails.
    Q_PROPERTY(QString display READ CurrentDisplay NOTIFY ArrowsChanged)

    /// \brief The speed the colour map's last colour stands for, m/s, and
    /// the steering slider's top.
    Q_PROPERTY(double maxSpeed READ MaxSpeed NOTIFY ArrowsChanged)

    /// \brief The colour map's stops, slowest first, as #rrggbb.
    Q_PROPERTY(QStringList legend READ Legend NOTIFY ArrowsChanged)

    /// \brief Side of the grid, m.
    Q_PROPERTY(double extent READ Extent NOTIFY ArrowsChanged)

    /// \brief Distance between two arrows, m.
    Q_PROPERTY(double spacing READ Spacing NOTIFY ArrowsChanged)

    /// \brief m of arrow per m/s.
    Q_PROPERTY(double scale READ Scale CONSTANT)

    /// \brief Draw on each vehicle the water past its hull, its track and
    /// its heading.
    Q_PROPERTY(bool onVehicles READ OnVehicles WRITE SetOnVehicles
               NOTIFY ArrowsChanged)

    /// \brief Constructor.
    public: OceanCurrentPanel();

    /// \brief Destructor.
    public: ~OceanCurrentPanel() override;

    // Documentation inherited.
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

    // Documentation inherited.
    public: void Update(const UpdateInfo &_info,
                        EntityComponentManager &_ecm) override;

    /// \brief Send the edited keys that differ from the recipe.
    /// \param[in] _edits The fields' text by key: speed, direction,
    /// water_level.
    public: Q_INVOKABLE void Apply(const QVariantMap &_edits);

    /// \brief Draw the field another way.
    /// \param[in] _display off, arrows or trails.
    public: Q_INVOKABLE void SetDisplay(const QString &_display);

    /// \brief Set the colour map's maximum speed from a field's text.
    /// \param[in] _text m/s.
    public: Q_INVOKABLE void SetMaxSpeed(const QString &_text);

    /// \brief Steer the field by hand towards a speed and the heading the
    /// flow goes towards, ramping there.
    /// \param[in] _speed m/s.
    /// \param[in] _flowHeading Degrees clockwise from north the flow goes
    /// towards, whatever the field's own convention.
    public: Q_INVOKABLE void SteerFlow(double _speed, double _flowHeading);

    /// \brief Set the side of the grid from a field's text.
    /// \param[in] _text m.
    public: Q_INVOKABLE void SetExtent(const QString &_text);

    /// \brief Set the distance between two arrows from a field's text.
    /// \param[in] _text m.
    public: Q_INVOKABLE void SetSpacing(const QString &_text);

    /// \brief The ocean current, for the panel.
    /// \return Its values.
    public: QVariantMap Current() const;

    /// \brief What became of the last change.
    /// \return A short sentence, empty when nothing was sent.
    public: QString Status() const;

    /// \brief How the field is drawn.
    /// \return off, arrows or trails.
    public: QString CurrentDisplay() const;

    /// \brief The colour map's maximum speed.
    /// \return m/s.
    public: double MaxSpeed() const;

    /// \brief The colour map's stops.
    /// \return #rrggbb, slowest first.
    public: QStringList Legend() const;

    /// \brief Side of the grid.
    /// \return m.
    public: double Extent() const;

    /// \brief Distance between two arrows.
    /// \return m.
    public: double Spacing() const;

    /// \brief m of arrow per m/s.
    /// \return The scale.
    public: double Scale() const;

    /// \brief Whether the vehicles' drawings are on.
    /// \return True if they are.
    public: bool OnVehicles() const;

    /// \brief Turn the vehicles' drawings on or off.
    /// \param[in] _on True for on.
    public: void SetOnVehicles(bool _on);

    /// \brief The current's values changed.
    signals: void CurrentChanged();

    /// \brief The status changed.
    signals: void StatusChanged();

    /// \brief An arrow setting changed.
    signals: void ArrowsChanged();

    /// \brief Private data pointer.
    private: std::unique_ptr<OceanCurrentPanelPrivate> dataPtr;
  };
}

#endif
