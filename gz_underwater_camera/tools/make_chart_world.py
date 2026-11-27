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

    make_chart_world.py --out chart.sdf --range 0.5 --depth 2 --jerlov 3C \
        [--visibility 4] [--width 1280 --height 720 --hfov 1.2] \
        [--chart-width 0.28] [--sun 0.8]

The sun shines from behind the camera onto the chart's face. A chart far
off in murky water is lost in the veil (in 3C, 2 m away, its patches
differ by about a degree), as it is in a real photo: shoot it close.

Writes chart.sdf and chart.json, the patches' boxes in its image. Run the
world headless and save one frame of its underwater image:

    gz sim -s -r chart.sdf &
    save_frame.py --topic /chart/camera/underwater/image --out sim.png
"""

import argparse
import json
import math
from pathlib import Path

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

# The Jerlov water types, as gz_water_optics has them (src/Jerlov.cc; a test
# keeps the two the same): absorption a, scattering b and the daylight's
# diffuse attenuation K_d, 1/m, red, green and blue.
TABLE = {
    'I': ((0.295, 0.046, 0.018), (0.00104, 0.00205, 0.00381), (0.302, 0.049, 0.022)),
    'IA': ((0.295, 0.0468, 0.0221), (0.00256, 0.00402, 0.00631), (0.303, 0.0503, 0.0264)),
    'IB': ((0.296, 0.0469, 0.0235), (0.0468, 0.0565, 0.068), (0.310, 0.0572, 0.0342)),
    'II': ((0.296, 0.0469, 0.0241), (0.288, 0.387, 0.504), (0.337, 0.0845, 0.062)),
    'III': ((0.297, 0.0507, 0.0388), (0.788, 1.06, 1.38), (0.390, 0.129, 0.124)),
    '1C': ((0.304, 0.068, 0.105), (0.293, 0.395, 0.514), (0.395, 0.121, 0.179)),
    '3C': ((0.307, 0.078, 0.154), (0.855, 1.15, 1.50), (0.456, 0.187, 0.319)),
    '5C': ((0.327, 0.127, 0.297), (1.070, 1.44, 1.87), (0.48, 0.277, 0.535)),
    '7C': ((0.367, 0.233, 0.542), (1.89, 2.54, 3.30), (0.600, 0.470, 0.924)),
    '9C': ((0.436, 0.430, 0.943), (2.51, 3.38, 4.39), (0.763, 0.826, 1.56)),
}
JERLOV = list(TABLE)

# The visibility, m, is this over the green attenuation.
VISIBILITY_CONSTANT = 4.8


def max_visibility():
    """Return the visibility of pure water (type I), m: none is clearer."""
    a, b, _ = TABLE['I']
    return VISIBILITY_CONSTANT / (a[1] + b[1])


def resolve(jerlov, visibility=None):
    """
    Return a water's attenuation c, veil b / c and K_d, per channel.

    As gz_water_optics resolves it: a visibility scales what the type has
    above pure water, so that 4.8 / c_green comes out at it.
    """
    a, b, kd = (list(v) for v in TABLE[jerlov])
    if visibility is not None:
        pa, pb, pkd = TABLE['I']
        c_pure = pa[1] + pb[1]
        scale = (VISIBILITY_CONSTANT / visibility - c_pure) / (a[1] + b[1] - c_pure)
        a = [p + (x - p) * scale for p, x in zip(pa, a)]
        b = [p + (x - p) * scale for p, x in zip(pb, b)]
        kd = [p + (x - p) * scale for p, x in zip(pkd, kd)]
    c = [x + y for x, y in zip(a, b)]
    return c, [y / x for x, y in zip(c, b)], kd


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
    <light type="directional" name="sun"><direction>1 0 -0.3</direction>
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
    p.add_argument('--range', type=float, default=0.5, help='camera to chart, m')
    p.add_argument('--depth', type=float, default=2.0, help='of camera and chart, m')
    p.add_argument('--jerlov', default='3C', type=str.upper, choices=JERLOV,
                   help='Jerlov water type')
    p.add_argument('--visibility', type=float, help='horizontal visibility, m')
    p.add_argument('--width', type=int, default=1280)
    p.add_argument('--height', type=int, default=720)
    p.add_argument('--hfov', type=float, default=1.2, help='radians')
    p.add_argument('--chart-width', type=float, default=0.28, help='m (a ColorChecker is 0.28)')
    p.add_argument('--sun', type=float, default=0.8, help='the sun light diffuse level')
    p.add_argument('--ambient', type=float, default=0.3, help='the ambient light level')
    a = p.parse_args(argv)
    clearest = max_visibility()
    if a.visibility is not None:
        if not (0 < a.visibility < clearest):
            p.error(f'--visibility must be over 0 and under {clearest:.1f} m, '
                    "pure water's")
        if a.jerlov == 'I':
            p.error('--visibility cannot change type I, which is pure water')
    if not (math.isfinite(a.range) and a.range > 0):
        p.error('--range must be a positive number of metres')
    if not (math.isfinite(a.depth) and a.depth > 0):
        p.error('--depth must be a positive number of metres')
    if not (0 < a.hfov < math.pi):
        p.error('--hfov must be between 0 and pi radians')
    if a.width < 1 or a.height < 1:
        p.error('--width and --height must be at least 1 pixel')
    sdf, patches = world(a)
    open(a.out, 'w').write(sdf)
    json_path = Path(a.out).with_suffix('.json')
    json.dump(patches, open(json_path, 'w'), indent=1)
    print(f'wrote {a.out} and {json_path}')


if __name__ == '__main__':
    main()
