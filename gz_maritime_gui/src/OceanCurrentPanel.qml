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

// The ocean current: steered by hand on the compass, set exactly in the fields, and
// drawn on the water as arrows or flow trails coloured by speed. The values
// shown are the world's, whoever changed them.
ScrollView {
  id: root
  Layout.minimumWidth: 300
  Layout.minimumHeight: 520
  anchors.fill: parent
  clip: true
  contentWidth: availableWidth

  property var field: _OceanCurrentPanel.current

  // A number for a field, without float noise.
  function show(v) {
    return (v === undefined) ? "" : String(Math.round(v * 1000) / 1000);
  }
  // A heading as three digits, as charts write it.
  function bearing(v) {
    var d = Math.round(v === undefined ? 0 : v) % 360;
    return ("00" + d).slice(-3) + "°";
  }

  // A new recipe puts its values back in the fields.
  function refresh() {
    speed.text = show(field.speed);
    direction.text = show(field.direction);
    waterLevel.text = show(field.waterLevel);
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
      visible: !root.field.available
      text: "This world has no ocean current system."
      color: "grey"
      Layout.topMargin: 6
      Layout.leftMargin: 8
    }

    // Steer by hand.
    ColumnLayout {
      visible: root.field.available
      Layout.fillWidth: true
      Layout.topMargin: 8
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      spacing: 4
      Compass {
        id: compass
        Layout.alignment: Qt.AlignHCenter
        heading: root.field.flowHeading !== undefined ?
            root.field.flowHeading : 0
        speed: root.field.speed !== undefined ? root.field.speed : 0
        maxSpeed: _OceanCurrentPanel.maxSpeed
        color: "#1a99ff"
        still: !(root.field.speed > 0)
        onSteered: function(s, h) { _OceanCurrentPanel.SteerFlow(s, h); }
      }
      Label {
        Layout.alignment: Qt.AlignHCenter
        text: root.show(root.field.speed) + " m/s · sets " +
            root.bearing(root.field.direction)
        font.pixelSize: 15
        font.bold: true
      }
      RowLayout {
        Layout.fillWidth: true
        Label { text: "0"; color: "grey"; font.pixelSize: 11 }
        Slider {
          id: speedSlider
          Layout.fillWidth: true
          from: 0
          to: _OceanCurrentPanel.maxSpeed
          onMoved: _OceanCurrentPanel.SteerFlow(value, compass.shownHeading)
          Binding on value {
            when: !speedSlider.pressed
            value: root.field.speed !== undefined ? root.field.speed : 0
            restoreMode: Binding.RestoreNone
          }
        }
        Label {
          text: root.show(_OceanCurrentPanel.maxSpeed)
          color: "grey"
          font.pixelSize: 11
        }
      }
      Label {
        Layout.fillWidth: true
        text: "Drag the arrow to turn the flow, slide or scroll for its " +
            "speed; the ocean current steps there."
        font.pixelSize: 11
        color: "grey"
        wrapMode: Text.WordWrap
        horizontalAlignment: Text.AlignHCenter
      }
    }

    // Set exactly.
    GridLayout {
      visible: root.field.available
      columns: 2
      Layout.fillWidth: true
      Layout.topMargin: 6
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      Label { text: "Speed (m/s)" }
      NumberField { id: speed }
      Label { text: "Sets towards (°)" }
      NumberField { id: direction }
      Label { text: "Water level (m)" }
      NumberField { id: waterLevel }
    }
    Label {
      visible: root.field.available
      text: "Direction it sets towards, clockwise from north: 90 flows east."
      font.pixelSize: 11
      color: "grey"
      wrapMode: Text.WordWrap
      Layout.fillWidth: true
      Layout.leftMargin: 8
    }
    RowLayout {
      visible: root.field.available
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
      visible: root.field.available
      text: "Model: " + root.field.model +
          (root.field.parameters ? "\n" + root.field.parameters : "")
      font.pixelSize: 11
      color: "grey"
      wrapMode: Text.WrapAnywhere
      Layout.fillWidth: true
      Layout.leftMargin: 8
    }

    // Draw on the water.
    RowLayout {
      Layout.fillWidth: true
      Layout.topMargin: 8
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      enabled: root.field.available
      Label { text: "Show" }
      ComboBox {
        id: display
        Layout.fillWidth: true
        model: ["Off", "Arrows", "Trails"]
        readonly property var names: ["off", "arrows", "trails"]
        currentIndex: Math.max(0, names.indexOf(_OceanCurrentPanel.display))
        onActivated: function(i) { _OceanCurrentPanel.SetDisplay(names[i]); }
      }
    }
    CheckBox {
      text: "On the vehicles: water past the hull, track, heading"
      checked: _OceanCurrentPanel.onVehicles
      enabled: root.field.available
      onToggled: _OceanCurrentPanel.onVehicles = checked
      Layout.leftMargin: 4
    }
    Legend {
      visible: _OceanCurrentPanel.display !== "off"
      Layout.fillWidth: true
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      colors: _OceanCurrentPanel.legend
      maxSpeed: _OceanCurrentPanel.maxSpeed
    }
    GridLayout {
      visible: _OceanCurrentPanel.display !== "off"
      columns: 2
      Layout.fillWidth: true
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      Label { text: "Top speed (m/s)" }
      NumberField {
        text: root.show(_OceanCurrentPanel.maxSpeed)
        onEditingFinished: _OceanCurrentPanel.SetMaxSpeed(text)
      }
      Label { text: "Area side (m)" }
      NumberField {
        text: root.show(_OceanCurrentPanel.extent)
        onEditingFinished: _OceanCurrentPanel.SetExtent(text)
      }
      Label {
        text: "Arrow spacing (m)"
        visible: _OceanCurrentPanel.display === "arrows"
      }
      NumberField {
        visible: _OceanCurrentPanel.display === "arrows"
        text: root.show(_OceanCurrentPanel.spacing)
        onEditingFinished: _OceanCurrentPanel.SetSpacing(text)
      }

    }
    Item { Layout.fillHeight: true }
  }
}
