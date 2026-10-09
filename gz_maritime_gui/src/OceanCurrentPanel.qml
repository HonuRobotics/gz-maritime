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

// The ocean current: its values, read from the recipe on the world, fields
// to change them, and the arrows that draw it.
ScrollView {
  id: root
  Layout.minimumWidth: 300
  Layout.minimumHeight: 380
  anchors.fill: parent
  clip: true
  contentWidth: availableWidth

  property var current: _OceanCurrentPanel.current

  // A number for a field, without float noise.
  function show(v) {
    return (v === undefined) ? "" : String(Math.round(v * 1000) / 1000);
  }

  // A new recipe puts its values back in the fields.
  function refresh() {
    speed.text = show(current.speed);
    direction.text = show(current.direction);
    waterLevel.text = show(current.waterLevel);
  }
  Connections {
    target: _OceanCurrentPanel
    function onCurrentChanged() { root.refresh(); }
  }
  Component.onCompleted: refresh()

  ColumnLayout {
    width: root.availableWidth
    spacing: 6

    Label {
      visible: !root.current.available
      text: "This world has no ocean current system."
      color: "grey"
      Layout.topMargin: 6
      Layout.leftMargin: 8
    }
    RowLayout {
      visible: root.current.available
      Layout.topMargin: 6
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      GridLayout {
        columns: 2
        Layout.fillWidth: true
        Label { text: "Speed (m/s)" }
        NumberField { id: speed }
        Label { text: "Sets towards (°)" }
        NumberField { id: direction }
        Label { text: "Water level (m)" }
        NumberField { id: waterLevel }
      }
      Compass {
        heading: root.current.flowHeading !== undefined ?
            root.current.flowHeading : 0
        color: "#1a99ff"
        still: !(root.current.speed > 0)
      }
    }
    Label {
      visible: root.current.available
      text: "Direction it sets towards, clockwise from north: 90 flows east."
      font.pixelSize: 11
      color: "grey"
      wrapMode: Text.WordWrap
      Layout.fillWidth: true
      Layout.leftMargin: 8
    }
    RowLayout {
      visible: root.current.available
      Layout.leftMargin: 8
      Button {
        text: "Apply"
        onClicked: _OceanCurrentPanel.Apply({
            "speed": speed.text,
            "direction": direction.text,
            "water_level": waterLevel.text})
      }
      Label {
        text: _OceanCurrentPanel.status
        wrapMode: Text.WordWrap
        Layout.fillWidth: true
      }
    }
    Label {
      visible: root.current.available
      text: "Model: " + root.current.model +
          (root.current.parameters ? "\n" + root.current.parameters : "")
      font.pixelSize: 11
      color: "grey"
      wrapMode: Text.WrapAnywhere
      Layout.fillWidth: true
      Layout.leftMargin: 8
    }

    CheckBox {
      text: "Arrows (blue), 1 m/s = " + root.show(_OceanCurrentPanel.scale) +
          " m"
      checked: _OceanCurrentPanel.showArrows
      enabled: root.current.available
      onToggled: _OceanCurrentPanel.showArrows = checked
      Layout.topMargin: 6
      Layout.leftMargin: 4
    }
    GridLayout {
      columns: 2
      Layout.fillWidth: true
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      Label { text: "Grid side (m)" }
      NumberField {
        text: root.show(_OceanCurrentPanel.extent)
        onEditingFinished: _OceanCurrentPanel.SetExtent(text)
      }
      Label { text: "Spacing (m)" }
      NumberField {
        text: root.show(_OceanCurrentPanel.spacing)
        onEditingFinished: _OceanCurrentPanel.SetSpacing(text)
      }
    }
    Item { Layout.fillHeight: true }
  }
}
