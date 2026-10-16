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

// A compass rose, north up, with an arrow along the flow: where the air or
// the water goes, whatever convention the numbers beside it follow. Drag the
// arrow to turn the flow, and turn the mouse wheel over it to change the
// speed; it emits steered() and shows the dragged heading until released,
// then follows the world again as the field steps there.
Item {
  id: compass
  // Degrees clockwise from north the flow goes towards, from the world.
  property real heading: 0
  // m/s, from the world, and the top of the speed range.
  property real speed: 0
  property real maxSpeed: 1
  property color color: "steelblue"
  // A still field has no direction to show: the arrow fades.
  property bool still: false
  property bool interactive: true
  // The user steers towards a speed and a flow heading.
  signal steered(real speed, real heading)

  property bool dragging: false
  property real dragHeading: 0
  readonly property real shownHeading: dragging ? dragHeading : heading

  implicitWidth: 120
  implicitHeight: 120

  function angleAt(x, y) {
    var a = Math.atan2(x - width / 2, -(y - height / 2)) * 180 / Math.PI;
    return (a + 360) % 360;
  }

  Rectangle {
    anchors.fill: parent
    radius: width / 2
    color: Qt.rgba(0.5, 0.5, 0.5, 0.06)
    border.color: mouse.containsMouse && compass.interactive ?
        compass.color : "grey"
    border.width: mouse.containsMouse && compass.interactive ? 2 : 1
  }
  // Ticks every 30 degrees, longer at the cardinal points.
  Repeater {
    model: 12
    Rectangle {
      width: 1
      height: index % 3 === 0 ? 8 : 4
      color: "grey"
      x: compass.width / 2
      y: 1
      transform: Rotation {
        origin.x: 0
        origin.y: compass.height / 2 - 1
        angle: index * 30
      }
    }
  }
  Repeater {
    model: ["N", "E", "S", "W"]
    Text {
      text: modelData
      font.pixelSize: 10
      font.bold: index === 0
      color: "grey"
      x: compass.width / 2 - width / 2 +
          Math.sin(index * Math.PI / 2) * (compass.width / 2 - 16)
      y: compass.height / 2 - height / 2 -
          Math.cos(index * Math.PI / 2) * (compass.height / 2 - 16)
    }
  }
  Canvas {
    id: arrow
    anchors.fill: parent
    rotation: compass.shownHeading
    opacity: compass.still && !compass.dragging ? 0.25 : 1.0
    onPaint: {
      var ctx = getContext("2d");
      ctx.reset();
      var cx = width / 2, cy = height / 2, r = width / 2 - 14;
      ctx.strokeStyle = compass.color;
      ctx.fillStyle = compass.color;
      ctx.lineWidth = 4;
      ctx.lineCap = "round";
      ctx.beginPath();
      ctx.moveTo(cx, cy + r * 0.8);
      ctx.lineTo(cx, cy - r + 12);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(cx, cy - r);
      ctx.lineTo(cx - 9, cy - r + 16);
      ctx.lineTo(cx + 9, cy - r + 16);
      ctx.closePath();
      ctx.fill();
      ctx.beginPath();
      ctx.arc(cx, cy, 4, 0, 2 * Math.PI);
      ctx.fill();
    }
  }
  onColorChanged: arrow.requestPaint()

  MouseArea {
    id: mouse
    anchors.fill: parent
    enabled: compass.interactive
    hoverEnabled: true
    cursorShape: compass.interactive ? Qt.PointingHandCursor : Qt.ArrowCursor
    onPressed: function(m) {
      compass.dragging = true;
      compass.dragHeading = compass.angleAt(m.x, m.y);
      compass.steered(compass.speed, compass.dragHeading);
    }
    onPositionChanged: function(m) {
      if (!pressed)
        return;
      compass.dragHeading = compass.angleAt(m.x, m.y);
      compass.steered(compass.speed, compass.dragHeading);
    }
    onReleased: compass.dragging = false
    onWheel: function(w) {
      var step = compass.maxSpeed / 20;
      var s = compass.speed + (w.angleDelta.y > 0 ? step : -step);
      compass.steered(Math.max(0, Math.min(compass.maxSpeed, s)),
                      compass.shownHeading);
    }
  }
}
