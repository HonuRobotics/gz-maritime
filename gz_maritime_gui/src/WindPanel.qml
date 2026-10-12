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

// The wind: steered by hand on the compass, set exactly in the fields, and
// drawn on the water as arrows or flow trails coloured by speed. The values
// shown are the world's, whoever changed them.
ScrollView {
  id: root
  Layout.minimumWidth: 300
  Layout.minimumHeight: 600
  anchors.fill: parent
  clip: true
  contentWidth: availableWidth

  property var field: _WindPanel.wind

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
    speedGust.text = show(field.speedGust);
    speedGustTime.text = show(field.speedGustTime);
    directionGust.text = show(field.directionGust);
    directionGustTime.text = show(field.directionGustTime);
  }
  Connections {
    target: _WindPanel
    function onWindChanged() { root.refresh(); }
  }
  Component.onCompleted: refresh()

  ColumnLayout {
    width: root.availableWidth
    spacing: 6

    Label {
      visible: !root.field.available
      text: "This world has no wind system."
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
        maxSpeed: _WindPanel.maxSpeed
        color: "#ff8c26"
        still: !(root.field.speed > 0)
        onSteered: function(s, h) { _WindPanel.SteerFlow(s, h); }
      }
      Label {
        Layout.alignment: Qt.AlignHCenter
        text: root.show(root.field.speed) + " m/s · from " +
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
          to: _WindPanel.maxSpeed
          onMoved: _WindPanel.SteerFlow(value, compass.shownHeading)
          Binding on value {
            when: !speedSlider.pressed
            value: root.field.speed !== undefined ? root.field.speed : 0
            restoreMode: Binding.RestoreNone
          }
        }
        Label {
          text: root.show(_WindPanel.maxSpeed)
          color: "grey"
          font.pixelSize: 11
        }
      }
      Label {
        Layout.fillWidth: true
        text: "Drag the arrow to turn the flow, slide or scroll for its " +
            "speed; the wind ramps there."
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
      Label { text: "Comes from (°)" }
      NumberField { id: direction }
      Label { text: "Speed gust (m/s)" }
      NumberField { id: speedGust }
      Label { text: "Speed gust time (s)" }
      NumberField { id: speedGustTime }
      Label { text: "Direction gust (°)" }
      NumberField { id: directionGust }
      Label { text: "Direction gust time (s)" }
      NumberField { id: directionGustTime }
    }
    Label {
      visible: root.field.available
      text: "Direction it comes from, clockwise from north: 270 blows east. "
          + "Speed at " + root.show(root.field.referenceHeight) + " m above the water."
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
        onClicked: _WindPanel.Apply({
            "speed": speed.text,
            "direction": direction.text,
            "speed_gust": speedGust.text,
            "speed_gust_time": speedGustTime.text,
            "direction_gust": directionGust.text,
            "direction_gust_time": directionGustTime.text})
      }
      Label {
        text: _WindPanel.status
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
        currentIndex: Math.max(0, names.indexOf(_WindPanel.display))
        onActivated: function(i) { _WindPanel.SetDisplay(names[i]); }
      }
    }
    CheckBox {
      text: "On the vehicles: apparent wind"
      checked: _WindPanel.onVehicles
      enabled: root.field.available
      onToggled: _WindPanel.onVehicles = checked
      Layout.leftMargin: 4
    }
    CheckBox {
      text: "Wind socks"
      visible: _WindPanel.hasSocks
      checked: _WindPanel.showSocks
      enabled: root.field.available
      onToggled: _WindPanel.showSocks = checked
      Layout.leftMargin: 4
    }
    Legend {
      visible: _WindPanel.display !== "off"
      Layout.fillWidth: true
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      colors: _WindPanel.legend
      maxSpeed: _WindPanel.maxSpeed
    }
    GridLayout {
      visible: _WindPanel.display !== "off"
      columns: 2
      Layout.fillWidth: true
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      Label { text: "Top speed (m/s)" }
      NumberField {
        text: root.show(_WindPanel.maxSpeed)
        onEditingFinished: _WindPanel.SetMaxSpeed(text)
      }
      Label { text: "Area side (m)" }
      NumberField {
        text: root.show(_WindPanel.extent)
        onEditingFinished: _WindPanel.SetExtent(text)
      }
      Label {
        text: "Arrow spacing (m)"
        visible: _WindPanel.display === "arrows"
      }
      NumberField {
        visible: _WindPanel.display === "arrows"
        text: root.show(_WindPanel.spacing)
        onEditingFinished: _WindPanel.SetSpacing(text)
      }
      Label { text: "Height (m)" }
      NumberField {
        text: root.show(_WindPanel.height)
        onEditingFinished: _WindPanel.SetHeight(text)
      }
    }
    Item { Layout.fillHeight: true }
  }
}
