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
#ifndef GZ_SIM_MARITIME_GUI_TEST_MENULOADER_HH_
#define GZ_SIM_MARITIME_GUI_TEST_MENULOADER_HH_

#include <gz/gui/Plugin.hh>

namespace gz::sim::maritime_gui
{
  /// \brief A test plugin that adds another plugin a while after the GUI
  /// starts, the way the plugin menu does (MainWindow::OnAddPlugin, with
  /// no configuration), since the menu cannot be clicked offscreen.
  ///
  ///   <plugin filename="MenuLoader" name="Menu loader">
  ///     <add>AcousticsPanel</add>   the plugin to add
  ///     <after>5</after>            wall seconds after this one loads
  ///   </plugin>
  class MenuLoader : public gui::Plugin
  {
    Q_OBJECT

    /// \brief Constructor.
    public: MenuLoader() = default;

    // Documentation inherited
    public: void LoadConfig(const tinyxml2::XMLElement *_pluginElem)
        override;
  };
}  // namespace gz::sim::maritime_gui

#endif  // GZ_SIM_MARITIME_GUI_TEST_MENULOADER_HH_
