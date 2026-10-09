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

// A compass rose with north up and an arrow along the flow: where the air
// or the water goes, whatever convention the number beside it follows.
Item {
  id: compass
  // Degrees clockwise from north the flow goes towards.
  property real heading: 0
  property color color: "steelblue"
  // A still field has no direction to show: the arrow fades.
  property bool still: false
  implicitWidth: 64
  implicitHeight: 64

  Rectangle {
    anchors.fill: parent
    radius: width / 2
    color: "transparent"
    border.color: "grey"
  }
  Text {
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.top: parent.top
    anchors.topMargin: 2
    text: "N"
    font.pixelSize: 10
    color: "grey"
  }
  Canvas {
    id: arrow
    anchors.fill: parent
    rotation: compass.heading
    opacity: compass.still ? 0.25 : 1.0
    onPaint: {
      var ctx = getContext("2d");
      ctx.reset();
      var cx = width / 2, cy = height / 2, r = width / 2 - 8;
      ctx.strokeStyle = compass.color;
      ctx.fillStyle = compass.color;
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.moveTo(cx, cy + r);
      ctx.lineTo(cx, cy - r + 8);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(cx, cy - r);
      ctx.lineTo(cx - 6, cy - r + 10);
      ctx.lineTo(cx + 6, cy - r + 10);
      ctx.closePath();
      ctx.fill();
    }
  }
  onColorChanged: arrow.requestPaint()
}
