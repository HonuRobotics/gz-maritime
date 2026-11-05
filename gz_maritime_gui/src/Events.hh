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
#ifndef GZ_SIM_MARITIME_GUI_EVENTS_HH_
#define GZ_SIM_MARITIME_GUI_EVENTS_HH_

#include <string>

#include <QEvent>

namespace gz::sim::maritime_gui::events
{
  /// \brief One panel tells the others it started steering a field, sent to
  /// the GUI's main window the way gz-gui's own events are, so a field panel
  /// drops an edit the user typed but never applied. The core library is
  /// Qt-free, so the event lives with the plugins.
  class FieldSteered : public QEvent
  {
    /// \brief Constructor.
    /// \param[in] _field The field's name on its topics: `wind` or
    /// `ocean_current`.
    public: explicit FieldSteered(const std::string &_field)
      : QEvent(kType), field(_field)
    {
    }

    /// \brief The event's type. A fixed number, as gz-gui's events use, so
    /// every plugin agrees on it whatever order they load in.
    public: static const QEvent::Type kType =
        QEvent::Type(QEvent::User + 0x4d47);

    /// \brief The field steered.
    /// \return Its name on its topics.
    public: const std::string &Field() const
    {
      return this->field;
    }

    /// \brief The field's name.
    private: std::string field;
  };
}

#endif  // GZ_SIM_MARITIME_GUI_EVENTS_HH_
