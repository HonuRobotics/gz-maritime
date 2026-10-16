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

// The colour map as a bar, from still to the maximum speed.
ColumnLayout {
  id: legend
  // #rrggbb, slowest first.
  property var colors: []
  property real maxSpeed: 1
  spacing: 0

  Canvas {
    id: bar
    Layout.fillWidth: true
    Layout.preferredHeight: 10
    onPaint: {
      var ctx = getContext("2d");
      ctx.reset();
      if (legend.colors.length === 0)
        return;
      var g = ctx.createLinearGradient(0, 0, width, 0);
      for (var i = 0; i < legend.colors.length; ++i)
        g.addColorStop(legend.colors.length === 1 ? 0 :
                       i / (legend.colors.length - 1), legend.colors[i]);
      ctx.fillStyle = g;
      ctx.fillRect(0, 0, width, height);
    }
    onWidthChanged: requestPaint()
  }
  onColorsChanged: bar.requestPaint()
  RowLayout {
    Layout.fillWidth: true
    Label { text: "0"; font.pixelSize: 10; color: "grey" }
    Item { Layout.fillWidth: true }
    Label {
      text: String(Math.round(legend.maxSpeed * 100) / 100) + " m/s"
      font.pixelSize: 10
      color: "grey"
    }
  }
}
