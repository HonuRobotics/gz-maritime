#!/usr/bin/env python3
# Copyright 2026 Honu Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Write a world that reproduces a colour chart photographed under water.

A 24-patch chart (the ColorChecker layout, 4 rows of 6) faces an underwater
camera at a given range and depth, in a given water. The world, run, gives
the simulated image to compare with the real one (chart_validation.py); the
patches' pixel boxes in that image are written beside it.

    make_chart_world.py --out chart.sdf --range 2 --depth 5 --jerlov 3C \
        [--visibility 4] [--width 1280 --height 720 --hfov 1.2] \
        [--chart-width 0.28] [--sun 0.8]

Writes chart.sdf and chart.json. Run the world headless and save one frame
of `chart/camera/underwater/image`:

    gz sim -s -r chart.sdf
"""

import argparse
import json
import math

# The chart's patches, as the ColorChecker's published sRGB values (D65),
# row by row.
PATCHES = [
    ('dark skin', (115, 82, 68)), ('light skin', (194, 150, 130)),
    ('blue sky', (98, 122, 157)), ('foliage', (87, 108, 67)),
    ('blue flower', (133, 128, 177)), ('bluish green', (103, 189, 170)),
    ('orange', (214, 126, 44)), ('purplish blue', (80, 91, 166)),
    ('moderate red', (193, 90, 99)), ('purple', (94, 60, 108)),
    ('yellow green', (157, 188, 64)), ('orange yellow', (224, 163, 46)),
    ('blue', (56, 61, 150)), ('green', (70, 148, 73)),
    ('red', (175, 54, 60)), ('yellow', (231, 199, 31)),
    ('magenta', (187, 86, 149)), ('cyan', (8, 133, 161)),
    ('white', (243, 243, 242)), ('neutral 8', (200, 200, 200)),
    ('neutral 6.5', (160, 160, 160)), ('neutral 5', (122, 122, 121)),
    ('neutral 3.5', (85, 85, 85)), ('black', (52, 52, 52)),
]
COLS, ROWS = 6, 4


def linear(c):
    c = c / 255.0
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def world(args):
    """Return the world's SDF and the patches' boxes in the camera's image."""
    pitch = args.chart_width / COLS
    size = pitch * 0.85
    fx = args.width / (2 * math.tan(args.hfov / 2))
    visuals, boxes = [], []
    for k, (name, rgb) in enumerate(PATCHES):
        row, col = divmod(k, COLS)
        # In the chart's frame: y to the camera's left, z up; patch 1 top left
        # as the camera sees it.
        y = (COLS / 2 - 0.5 - col) * pitch
        z = (ROWS / 2 - 0.5 - row) * pitch
        lin = ' '.join(f'{linear(c):.4f}' for c in rgb) + ' 1'
        visuals.append(
            f'        <visual name="p{k}"><pose>0 {y:.4f} {z:.4f} 0 0 0</pose>'
            f'<geometry><box><size>0.004 {size:.4f} {size:.4f}</size></box>'
            f'</geometry><material><ambient>{lin}</ambient>'
            f'<diffuse>{lin}</diffuse><specular>0 0 0 1</specular></material>'
            '</visual>')
        # Its box in the image: a pinhole looking along +x at the chart.
        u0 = args.width / 2 - fx * (y + size / 2) / args.range
        u1 = args.width / 2 - fx * (y - size / 2) / args.range
        v0 = args.height / 2 - fx * (z + size / 2) / args.range
        v1 = args.height / 2 - fx * (z - size / 2) / args.range
        boxes.append({'name': name,
                      'box': [round(u0, 1), round(v0, 1), round(u1, 1), round(v1, 1)]})
    water = f'<jerlov>{args.jerlov}</jerlov>'
    if args.visibility:
        water += f'<visibility>{args.visibility}</visibility>'
    sdf = f"""<?xml version="1.0"?>
<!-- A colour chart {args.range} m from an underwater camera, {args.depth} m deep,
     Jerlov {args.jerlov}. Written by make_chart_world.py. -->
<sdf version="1.9">
  <world name="chart">
    <physics name="p" type="ignored"><max_step_size>0.01</max_step_size></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-maritime-water-optics-system"
            name="gz::sim::maritime::WaterOptics">{water}</plugin>
    <scene>
      <ambient>{args.ambient} {args.ambient} {args.ambient} 1</ambient>
      <background>0 0 0 1</background>
    </scene>
    <light type="directional" name="sun"><direction>0.2 0.1 -1</direction>
      <diffuse>{args.sun} {args.sun} {args.sun} 1</diffuse><specular>0 0 0 1</specular></light>
    <model name="chart">
      <static>true</static>
      <pose>{args.range} 0 {-args.depth} 0 0 0</pose>
      <link name="link">
{chr(10).join(visuals)}
      </link>
    </model>
    <model name="camera">
      <static>true</static>
      <pose>0 0 {-args.depth} 0 0 0</pose>
      <link name="link">
        <sensor name="camera" type="rgbd_camera">
          <topic>chart/camera</topic>
          <update_rate>5</update_rate>
          <always_on>1</always_on>
          <camera>
            <horizontal_fov>{args.hfov}</horizontal_fov>
            <image><width>{args.width}</width><height>{args.height}</height></image>
            <clip><near>0.05</near><far>100</far></clip>
            <depth_camera><clip><near>0.05</near><far>100</far></clip></depth_camera>
          </camera>
          <plugin filename="gz-maritime-underwater-camera-system"
                  name="gz::sim::maritime::UnderwaterCamera"/>
        </sensor>
      </link>
    </model>
  </world>
</sdf>
"""
    return sdf, {'patches': boxes}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--out', required=True, help='the world file to write (.sdf)')
    p.add_argument('--range', type=float, default=2.0, help='camera to chart, m')
    p.add_argument('--depth', type=float, default=5.0, help='of camera and chart, m')
    p.add_argument('--jerlov', default='3C', help='Jerlov water type')
    p.add_argument('--visibility', type=float, help='horizontal visibility, m')
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--hfov', type=float, default=1.2, help='radians')
    p.add_argument('--chart-width', type=float, default=0.28, help='m (a ColorChecker is 0.28)')
    p.add_argument('--sun', type=float, default=0.8, help='the sun light diffuse level')
    p.add_argument('--ambient', type=float, default=0.3, help='the ambient light level')
    a = p.parse_args(argv)
    sdf, patches = world(a)
    open(a.out, 'w').write(sdf)
    json_path = a.out.rsplit('.', 1)[0] + '.json'
    json.dump(patches, open(json_path, 'w'), indent=1)
    print(f'wrote {a.out} and {json_path}')


if __name__ == '__main__':
    main()
