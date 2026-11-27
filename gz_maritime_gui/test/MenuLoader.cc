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
#include "MenuLoader.hh"

#include <string>

#include <QString>
#include <QTimer>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <tinyxml2.h>

namespace gz::sim::maritime_gui
{
//////////////////////////////////////////////////
void MenuLoader::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  std::string add;
  double after{5.0};
  if (_pluginElem)
  {
    if (const auto *e = _pluginElem->FirstChildElement("add"))
      add = e->GetText() ? e->GetText() : "";
    if (const auto *e = _pluginElem->FirstChildElement("after"))
      e->QueryDoubleText(&after);
  }
  if (add.empty())
  {
    gzerr << "MenuLoader: no <add>\n";
    return;
  }
  QTimer::singleShot(static_cast<int>(after * 1000.0), this, [add]()
  {
    auto *window = gui::App()->findChild<gui::MainWindow *>();
    if (!window)
    {
      gzerr << "MenuLoader: no main window\n";
      return;
    }
    gzdbg << "MenuLoader: adding [" << add << "] as the menu does\n";
    window->OnAddPlugin(QString::fromStdString(add));
  });
}
}  // namespace gz::sim::maritime_gui

GZ_ADD_PLUGIN(gz::sim::maritime_gui::MenuLoader, gz::gui::Plugin)
