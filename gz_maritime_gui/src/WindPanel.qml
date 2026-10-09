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

// The wind: its values, read from the recipe on the world, fields to change
// them, and the arrows that draw it.
ScrollView {
  id: root
  Layout.minimumWidth: 300
  Layout.minimumHeight: 480
  anchors.fill: parent
  clip: true
  contentWidth: availableWidth

  property var wind: _WindPanel.wind

  // A number for a field, without float noise.
  function show(v) {
    return (v === undefined) ? "" : String(Math.round(v * 1000) / 1000);
  }

  // A new recipe puts its values back in the fields.
  function refresh() {
    speed.text = show(wind.speed);
    direction.text = show(wind.direction);
    speedGust.text = show(wind.speedGust);
    speedGustTime.text = show(wind.speedGustTime);
    directionGust.text = show(wind.directionGust);
    directionGustTime.text = show(wind.directionGustTime);
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
      visible: !root.wind.available
      text: "This world has no wind system."
      color: "grey"
      Layout.topMargin: 6
      Layout.leftMargin: 8
    }
    RowLayout {
      visible: root.wind.available
      Layout.topMargin: 6
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      GridLayout {
        columns: 2
        Layout.fillWidth: true
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
      Compass {
        heading: root.wind.flowHeading !== undefined ?
            root.wind.flowHeading : 0
        color: "#ffd933"
        still: !(root.wind.speed > 0)
        Layout.alignment: Qt.AlignTop
      }
    }
    Label {
      visible: root.wind.available
      text: "Direction it comes from, clockwise from north: 270 blows " +
          "east. Speed at " + root.show(root.wind.referenceHeight) +
          " m above the water."
      font.pixelSize: 11
      color: "grey"
      wrapMode: Text.WordWrap
      Layout.fillWidth: true
      Layout.leftMargin: 8
    }
    RowLayout {
      visible: root.wind.available
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
      visible: root.wind.available
      text: "Model: " + root.wind.model
      font.pixelSize: 11
      color: "grey"
      Layout.leftMargin: 8
    }

    CheckBox {
      text: "Arrows (yellow), 1 m/s = " + root.show(_WindPanel.scale) + " m"
      checked: _WindPanel.showArrows
      enabled: root.wind.available
      onToggled: _WindPanel.showArrows = checked
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
        text: root.show(_WindPanel.extent)
        onEditingFinished: _WindPanel.SetExtent(text)
      }
      Label { text: "Spacing (m)" }
      NumberField {
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
