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
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

// A field for a number, with a decimal point or a decimal comma whatever
// the locale: the plugin parses the text, so the locale's validator, which
// takes only its own separator, is not used.
TextField {
  Layout.fillWidth: true
  selectByMouse: true
  inputMethodHints: Qt.ImhFormattedNumbersOnly
  validator: RegularExpressionValidator {
    regularExpression: /^\s*[-+]?[0-9]*([.,][0-9]*)?\s*$/
  }
}
