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

// Named conditions for the ocean current and the wind: a button each, and a
// demo that steps through them.
ColumnLayout {
  id: root
  Layout.minimumWidth: 280
  Layout.minimumHeight: 200
  anchors.fill: parent
  anchors.margins: 8
  spacing: 8

  GridLayout {
    columns: 2
    Layout.fillWidth: true
    columnSpacing: 6
    rowSpacing: 6
    Repeater {
      model: _ScenarioPanel.presets
      Button {
        Layout.fillWidth: true
        Layout.preferredHeight: 44
        text: modelData
        highlighted: index === _ScenarioPanel.active
        font.bold: index === _ScenarioPanel.active
        onClicked: _ScenarioPanel.ApplyPreset(index)
      }
    }
  }
  RowLayout {
    Layout.fillWidth: true
    Switch {
      text: "Demo"
      checked: _ScenarioPanel.demo
      onToggled: _ScenarioPanel.SetDemo(checked)
    }
    Label {
      text: _ScenarioPanel.status
      wrapMode: Text.WordWrap
      Layout.fillWidth: true
    }
  }
  Label {
    Layout.fillWidth: true
    text: "A preset steers the current and the wind to it; the demo steps " +
        "through them, staying on each a while of simulation time."
    font.pixelSize: 11
    color: "grey"
    wrapMode: Text.WordWrap
  }
  Item { Layout.fillHeight: true }
}
