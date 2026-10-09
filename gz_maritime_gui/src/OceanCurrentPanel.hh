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
  /// It draws the current as a grid of blue arrows just above the water,
  /// sampled from the recipe with OceanCurrentSampler, as a LINE_LIST marker
  /// through MarkerManager, redrawn when the recipe changes.
  ///
  /// A plugin added from the menu after the world is up missed the recipe;
  /// it then asks the world's state service once.
  ///
  /// Configuration, all optional:
  ///
  /// * `<show_arrows>`: draw the arrows, default false.
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

    /// \brief Draw the arrows.
    Q_PROPERTY(bool showArrows READ ShowArrows WRITE SetShowArrows
               NOTIFY ArrowsChanged)

    /// \brief Side of the grid, m.
    Q_PROPERTY(double extent READ Extent NOTIFY ArrowsChanged)

    /// \brief Distance between two arrows, m.
    Q_PROPERTY(double spacing READ Spacing NOTIFY ArrowsChanged)

    /// \brief m of arrow per m/s.
    Q_PROPERTY(double scale READ Scale CONSTANT)

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

    /// \brief Whether the arrows are drawn.
    /// \return True if they are.
    public: bool ShowArrows() const;

    /// \brief Draw the arrows or not.
    /// \param[in] _show True to draw them.
    public: void SetShowArrows(bool _show);

    /// \brief Side of the grid.
    /// \return m.
    public: double Extent() const;

    /// \brief Distance between two arrows.
    /// \return m.
    public: double Spacing() const;

    /// \brief m of arrow per m/s.
    /// \return The scale.
    public: double Scale() const;

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
