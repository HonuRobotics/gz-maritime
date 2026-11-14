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

// The acoustic pingers and what the receivers hear: each pinger with its
// colour, settings and an on/off switch; each receiver with its band, which
// can be retuned, and its last reading; and what to draw on the scene.
ScrollView {
  id: root
  Layout.minimumWidth: 300
  Layout.minimumHeight: 360
  anchors.fill: parent
  clip: true
  contentWidth: availableWidth

  function show(v) {
    return (v === undefined) ? "" : String(Math.round(v * 1000) / 1000);
  }

  ColumnLayout {
    width: root.availableWidth
    spacing: 6

    Label {
      text: _AcousticsPanel.status
      color: "grey"
      wrapMode: Text.WordWrap
      Layout.fillWidth: true
      Layout.topMargin: 6
      Layout.leftMargin: 8
      Layout.rightMargin: 8
    }

    // ---------------------------------------------------------------- pingers
    Label {
      visible: _AcousticsPanel.pingers.length > 0
      text: "Pingers"
      font.bold: true
      Layout.leftMargin: 8
    }
    Repeater {
      model: _AcousticsPanel.pingers
      RowLayout {
        Layout.fillWidth: true
        Layout.leftMargin: 8
        Layout.rightMargin: 8
        spacing: 6
        Rectangle {
          width: 12
          height: 12
          radius: 6
          color: modelData.active ? modelData.colour : "transparent"
          border.color: modelData.colour
          border.width: 2
        }
        ColumnLayout {
          Layout.fillWidth: true
          spacing: 0
          Label {
            text: modelData.name + "  " + root.show(modelData.frequency) + " kHz"
            font.bold: true
          }
          Label {
            text: "every " + root.show(modelData.period) + " s, at +" +
                root.show(modelData.offset) + " s, reach " +
                (modelData.reach > 0 ? root.show(modelData.reach) + " m" : "unlimited")
            font.pixelSize: 11
            color: "grey"
          }
        }
        Switch {
          checked: modelData.active
          onToggled: _AcousticsPanel.SetActive(modelData.name, checked)
        }
      }
    }

    // -------------------------------------------------------------- receivers
    Label {
      visible: _AcousticsPanel.receivers.length > 0
      text: "Receivers"
      font.bold: true
      Layout.leftMargin: 8
      Layout.topMargin: 4
    }
    Repeater {
      model: _AcousticsPanel.receivers
      ColumnLayout {
        Layout.fillWidth: true
        Layout.leftMargin: 8
        Layout.rightMargin: 8
        spacing: 2
        RowLayout {
          Layout.fillWidth: true
          spacing: 6
          Rectangle {
            width: 12
            height: 12
            radius: 2
            color: modelData.frequency > 0 ? modelData.colour : "grey"
          }
          Label {
            text: modelData.name
            font.bold: true
            elide: Text.ElideMiddle
            Layout.fillWidth: true
          }
          NumberField {
            id: tune
            Layout.fillWidth: false
            Layout.preferredWidth: 70
            text: root.show(modelData.frequency)
            onAccepted: {
              if (_AcousticsPanel.SetReceiverFrequency(modelData.topic, text))
                edited = false;
            }
          }
          Label { text: "kHz" }
        }
        Label {
          text: (modelData.frequency > 0 ?
                    "± " + root.show(modelData.bandwidth / 2) + " kHz: " :
                    "every frequency: ") +
                (_AcousticsPanel.readings[modelData.topic] || "nothing heard")
          font.pixelSize: 11
          color: "grey"
          wrapMode: Text.WordWrap
          Layout.fillWidth: true
        }
      }
    }

    // ---------------------------------------------------------------- drawing
    Label {
      text: "Draw"
      font.bold: true
      Layout.leftMargin: 8
      Layout.topMargin: 4
    }
    ColumnLayout {
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      spacing: 0
      RowLayout {
        Label { text: "Reach and pings on" }
        ComboBox {
          id: planeBox
          Layout.fillWidth: true
          textRole: "label"
          valueRole: "key"
          model: [
            { "key": "water", "label": "the water" },
            { "key": "depth", "label": "a depth" },
            { "key": "receiver", "label": "a receiver's depth" },
            { "key": "3d", "label": "spheres, in 3D" }
          ]
          currentIndex: Math.max(0, indexOfValue(_AcousticsPanel.plane))
          onActivated: _AcousticsPanel.SetPlane(currentValue)
        }
      }
      RowLayout {
        Layout.leftMargin: 28
        visible: _AcousticsPanel.plane === "depth"
        NumberField {
          Layout.fillWidth: false
          Layout.preferredWidth: 60
          text: root.show(_AcousticsPanel.planeDepth)
          onAccepted: {
            if (_AcousticsPanel.SetPlaneDepth(text))
              edited = false;
          }
        }
        Label { text: "m under the water" }
      }
      ComboBox {
        Layout.leftMargin: 28
        Layout.fillWidth: true
        visible: _AcousticsPanel.plane === "receiver"
        textRole: "name"
        valueRole: "topic"
        model: _AcousticsPanel.receivers
        currentIndex: Math.max(0, indexOfValue(_AcousticsPanel.planeReceiver))
        onActivated: _AcousticsPanel.SetPlaneReceiver(currentValue)
      }
      Label {
        Layout.leftMargin: 28
        text: "Drawing " + _AcousticsPanel.planeText +
            (_AcousticsPanel.plane === "water" ? "" :
             ": under the drawn sea, seen from under the water" +
             " or with the water hidden.")
        font.pixelSize: 11
        color: "grey"
        wrapMode: Text.WordWrap
        Layout.fillWidth: true
      }
      CheckBox {
        text: "Reach"
        checked: _AcousticsPanel.showReach
        onToggled: _AcousticsPanel.showReach = checked
      }
      CheckBox {
        text: "Pings spreading"
        checked: _AcousticsPanel.showPings
        onToggled: _AcousticsPanel.showPings = checked
      }
      RowLayout {
        Layout.leftMargin: 28
        enabled: _AcousticsPanel.showPings
        Label { text: "slowed down" }
        NumberField {
          Layout.fillWidth: false
          Layout.preferredWidth: 60
          text: root.show(_AcousticsPanel.slowDown)
          onAccepted: {
            if (_AcousticsPanel.SetSlowDown(text))
              edited = false;
          }
        }
        Label { text: "times" }
      }
      CheckBox {
        text: "Bearings heard"
        checked: _AcousticsPanel.showReadings
        onToggled: _AcousticsPanel.showReadings = checked
      }
    }
    Label {
      Layout.fillWidth: true
      Layout.leftMargin: 8
      Layout.rightMargin: 8
      text: "Colour by frequency: 25 kHz orange to 40 kHz violet. A pinger " +
          "that is off is grey and dashed; an invalid reading is red."
      font.pixelSize: 11
      color: "grey"
      wrapMode: Text.WordWrap
    }
    Item { Layout.fillHeight: true }
  }
}
