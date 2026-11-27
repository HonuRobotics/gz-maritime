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
Save one image of a Gazebo image topic as a PNG.

    save_frame.py --topic /chart/camera/underwater/image --out sim.png \
        [--after 1.0] [--timeout 60]

Waits for the first image stamped at or after --after seconds of simulation
time, so the scene has settled, and writes it. Exits 1 if none comes within
--timeout seconds of wall time.
"""

import argparse
import sys
import threading

import numpy as np

# Channels of the pixel formats a camera publishes, by name.
CHANNELS = {'L_INT8': 1, 'RGB_INT8': 3, 'RGBA_INT8': 4, 'BGR_INT8': 3,
            'BGRA_INT8': 4}


def to_array(msg):
    """Return an image message as an RGB array of bytes."""
    formats = type(msg).DESCRIPTOR.fields_by_name['pixel_format_type'].enum_type
    fmt = formats.values_by_number[msg.pixel_format_type].name
    if fmt not in CHANNELS:
        raise ValueError(f'pixel format {fmt} is not supported')
    n = CHANNELS[fmt]
    step = msg.step or msg.width * n
    rows = np.frombuffer(msg.data, dtype=np.uint8)[:step * msg.height]
    img = rows.reshape(msg.height, step)[:, :msg.width * n].reshape(
        msg.height, msg.width, n)
    if fmt.startswith('BGR'):
        img = img[..., [2, 1, 0]]
    elif n == 1:
        img = np.repeat(img, 3, axis=2)
    return np.ascontiguousarray(img[..., :3])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--topic', required=True, help='the image topic')
    p.add_argument('--out', required=True, help='the PNG to write')
    p.add_argument('--after', type=float, default=1.0,
                   help='simulation time of the first image to keep, s')
    p.add_argument('--timeout', type=float, default=60.0,
                   help='wall time to wait, s')
    a = p.parse_args(argv)

    from gz.msgs.image_pb2 import Image
    from gz.transport import Node
    from PIL import Image as Png

    got = threading.Event()
    kept = []

    def on_image(msg):
        stamp = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec
        if not got.is_set() and stamp >= a.after:
            kept.append(msg)
            got.set()

    node = Node()
    if not node.subscribe(Image, a.topic, on_image):
        print(f'cannot subscribe to {a.topic}', file=sys.stderr)
        return 1
    if not got.wait(a.timeout):
        print(f'no image on {a.topic} after {a.after} s of simulation, '
              f'in {a.timeout} s', file=sys.stderr)
        return 1
    Png.fromarray(to_array(kept[0])).save(a.out)
    print(f'wrote {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
