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
Regenerate the Gazebo screenshots used in the documentation.

Each figure is rendered by a camera sensor in a copy of an installed world
(open_water.sdf unless the figure names another), with the vehicles included
in the world, and saved from the camera's image topic. One change to the
copy of open water, for the picture only:

* its deep-ocean ambient light is set to the Gazebo GUI's 0.4 grey, so
  vehicles keep the colours the GUI shows (that ambient tints camera sensors
  cyan). The site worlds already light their terrain with a white ambient.

Needs a sourced workspace with gz-maritime and bluerobotics_models built, a
GPU with an X display, the gz-transport Python bindings
(python3-gz-transport15 on Jetty, python3-gz-transport14 on Ionic) and, for
the site figures, the Sydney Regatta and Benderson Park terrains in the Fuel
cache (run each world once).

    python3 docs/_tools/gazebo_figures.py            # every figure
    python3 docs/_tools/gazebo_figures.py first-simulation
"""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

from ament_index_python.packages import get_package_share_directory
from PIL import Image as PILImage
from PIL import ImageDraw
from PIL import ImageFont

try:
    from gz.msgs.image_pb2 import Image
    from gz.transport import Node
except ImportError:
    from gz.msgs11.image_pb2 import Image
    from gz.transport14 import Node

DOCS = Path(__file__).resolve().parent.parent
IMAGE_TOPIC = '/world/default/model/docs_camera/link/link/sensor/camera/image'

FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

# name: output file, world (open_water unless given), vehicles to include
# (uri, name, and an optional pose), sea state, camera pose and FOV, and the
# simulation times [s] to capture. The sea has a fixed seed, so a given time
# gives the same picture on every run. Several times make one image of
# labelled panels side by side.
USV = 'model://custom_usv'
SITE_FOV = 1.05  # the GUI's own field of view: the figure is the opening view
FIGURES = {
    'ocean': {
        # Low and looking out to the horizon: from the GUI's own, steeper
        # camera a calm sea is a flat blue field.
        'out': DOCS / 'getting-started' / 'images' / 'open-water.jpg',
        'vehicles': [],
        'sea_state': 1,
        'camera': '0 0 2.5 0 0.12 0',
        'fov': 1.2,
        'at': [10],
    },
    'first-simulation': {
        'out': DOCS / 'getting-started' / 'images' / 'custom-usv.jpg',
        'vehicles': [('model://custom_usv', 'custom_usv')],
        'sea_state': 1,
        'camera': '1.85 -1.5 0.85 0 0.33 2.46',
        'fov': 0.75,
        'at': [10],
    },
    'waves-through-hulls': {
        'out': DOCS / 'how-to' / 'images' / 'waves-through-hulls.jpg',
        'vehicles': [('model://custom_usv', 'custom_usv')],
        'sea_state': 3,
        'camera': '1.85 -1.5 1.05 0 0.40 2.46',
        'fov': 0.8,
        'at': [9, 21],
        'labels': ['Trough: the hulls hang above the water',
                   'Crest: the water covers the hulls'],
    },
    'blueboat': {
        'out': DOCS / 'vehicles' / 'images' / 'blueboat.jpg',
        'vehicles': [('model://blueboat', 'blueboat')],
        'sea_state': 1,
        'camera': '2.1 -1.7 0.95 0 0.33 2.46',
        'fov': 0.75,
        'at': [10],
    },
    # The four site worlds from their opening views, with the custom USV at
    # the start point each world's header recommends. A 16:9 sensor sees
    # less sky than the GUI's taller viewport, so two cameras pitch up a
    # little from the world's pose to keep the hills in.
    'sydney-regatta': {
        'out': DOCS / 'how-to' / 'images' / 'sydney-regatta.jpg',
        'world': 'sydney_regatta',
        'vehicles': [(USV, 'usv', '-532 162 0 0 0 1')],
        'sea_state': 1,
        'camera': '-478.1 148.2 13.2 0 0.25 2.94',
        'fov': SITE_FOV,
        'at': [10],
    },
    'benderson-park': {
        'out': DOCS / 'how-to' / 'images' / 'benderson-park.jpg',
        'world': 'benderson_park',
        'vehicles': [(USV, 'usv', '0 0 0 0 0 1.57')],
        'sea_state': 1,
        'camera': '-193 1175 30 0 0.12 -1.18',
        'fov': SITE_FOV,
        'at': [10],
    },
    'sand-island': {
        'out': DOCS / 'how-to' / 'images' / 'sand-island.jpg',
        'world': 'sand_island',
        'vehicles': [(USV, 'usv', '158 108 0 0 0 -2.76')],
        'sea_state': 2,
        'camera': '173 122 4 0 0.22 -1.2',
        'fov': SITE_FOV,
        'at': [10],
    },
    # The multi vehicle demo at Sydney: the boat off the start point, the
    # ROV under the surface ahead of it, the X500 on the pad.
    'several-vehicles': {
        'out': DOCS / 'how-to' / 'images' / 'several-vehicles.jpg',
        'world': 'sydney_regatta',
        'vehicles': [('model://blueboat', 'blueboat', '-532 162 0.05 0 0 1'),
                     ('model://bluerov2', 'bluerov2', '-528.8 167 -1 0 0 1'),
                     ('model://x500', 'x500', '-540 168 1.25 0 0 0')],
        'sea_state': 1,
        'camera': '-522 152 4 0 0.2 2.35',
        'fov': 1.0,
        'at': [10],
    },
    'la-spezia': {
        'out': DOCS / 'how-to' / 'images' / 'la-spezia.jpg',
        'world': 'la_spezia',
        'vehicles': [(USV, 'usv', '10 -372 0 0 0 0.3816')],
        'sea_state': 2,
        'camera': '200 -480 55 0 0.2 -2.65',
        'fov': SITE_FOV,
        'at': [10],
    },
}

CAMERA = """
    <model name="docs_camera">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <update_rate>4</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>{fov}</horizontal_fov>
            <image><width>1600</width><height>900</height></image>
            <clip><near>0.05</near><far>1000</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


def figure_world(_figure):
    """Return the figure's world with its vehicles, camera and tweaks."""
    world = (Path(get_package_share_directory('kai_gazebo'))
             / 'worlds' / f'{_figure.get("world", "open_water")}.sdf').read_text()
    world = world.replace('<ambient>0.0 1.0 1.0</ambient>',
                          '<ambient>0.4 0.4 0.4</ambient>', 1)
    world = world.replace('<sea_state>1</sea_state>',
                          f'<sea_state>{_figure["sea_state"]}</sea_state>', 1)
    extra = ''.join(
        f'<include><uri>{v[0]}</uri><name>{v[1]}</name>'
        + (f'<pose>{v[2]}</pose>' if len(v) > 2 else '') + '</include>'
        for v in _figure['vehicles'])
    extra += CAMERA.format(pose=_figure['camera'], fov=_figure['fov'])
    return world.replace('</world>', extra + '</world>', 1)


def capture(_name, _figure):
    """Run the figure's world and return the first frame at each capture time."""
    targets = _figure['at']
    frames = {}

    def on_image(_msg):
        stamp = _msg.header.stamp.sec + _msg.header.stamp.nsec / 1e9
        for target in targets:
            if target not in frames and stamp >= target:
                frames[target] = _msg

    with tempfile.TemporaryDirectory() as tmp:
        world_file = Path(tmp) / f'{_name}.sdf'
        world_file.write_text(figure_world(_figure))
        node = Node()
        node.subscribe(Image, IMAGE_TOPIC, on_image)
        server = subprocess.Popen(['gz', 'sim', '-s', '-r', str(world_file)],
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
        try:
            deadline = time.time() + 240
            while len(frames) < len(targets) and time.time() < deadline:
                time.sleep(0.5)
        finally:
            server.terminate()
            server.wait(timeout=30)

    if len(frames) < len(targets):
        sys.exit(f'{_name}: no frame for sim times '
                 f'{sorted(set(targets) - set(frames))} within 240 s')
    return [PILImage.frombytes('RGB', (m.width, m.height), m.data)
            for m in (frames[t] for t in targets)]


def compose(_images, _labels):
    """Return one 1200 px wide image: a single frame, or labelled panels."""
    if len(_images) == 1:
        return _images[0].resize((1200, 675), PILImage.LANCZOS)
    width = 1200 // len(_images)
    height = width * 9 // 16
    bar = 48
    sheet = PILImage.new('RGB', (width * len(_images), height + bar), 'white')
    draw = ImageDraw.Draw(sheet)
    font = ImageFont.truetype(FONT, 20)
    for index, (image, label) in enumerate(zip(_images, _labels)):
        sheet.paste(image.resize((width, height), PILImage.LANCZOS),
                    (index * width, 0))
        draw.text((index * width + width // 2, height + bar // 2), label,
                  font=font, fill=(31, 41, 51), anchor='mm')
    return sheet


def render(_name, _figure):
    """Capture a figure and save it next to the page that uses it."""
    images = capture(_name, _figure)
    image = compose(images, _figure.get('labels', []))
    _figure['out'].parent.mkdir(parents=True, exist_ok=True)
    image.save(_figure['out'], quality=88, optimize=True)
    print(f'{_name}: {_figure["out"].relative_to(DOCS)}')


def main():
    """Render the figures named on the command line, or all of them."""
    # One private partition for the whole run, set before the first Node:
    # gz-transport reads it once, and it keeps a stray simulation out.
    os.environ['GZ_PARTITION'] = f'docs_figures_{uuid.uuid4().hex[:8]}'
    names = sys.argv[1:] or list(FIGURES)
    for name in names:
        render(name, FIGURES[name])


if __name__ == '__main__':
    main()
