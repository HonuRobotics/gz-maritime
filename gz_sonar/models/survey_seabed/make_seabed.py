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
Write the survey world's seabed as meshes/seabed.glb.

    make_seabed.py [height x y ...]

A gently undulating seabed about 8 m down, 120 m by 80 m at a 1 m grid.
The file is glTF-native, +Y up and +Z forward, so the model's visual turns
it into Gazebo's frame (roll 90 degrees, then yaw 90 degrees). With
arguments it prints the seabed's height at each (x, y) instead, for placing
targets on it.
"""

import json
import math
import pathlib
import struct
import sys

X0, X1, Y0, Y1, STEP = -40.0, 80.0, -40.0, 40.0, 1.0


def height(x, y):
    """Return the seabed's z at Gazebo x and y, m: swells of about 1 m."""
    return (-8.0 + 0.6 * math.sin(2 * math.pi * x / 25.0) * math.cos(2 * math.pi * y / 40.0)
            + 0.3 * math.sin(2 * math.pi * (x + y) / 13.0))


def to_gltf(x, y, z):
    """Gazebo (x forward, y left, z up) to glTF (+X left, +Y up, +Z forward)."""
    return (y, z, x)


def main():
    if len(sys.argv) > 1:
        v = [float(a) for a in sys.argv[1:]]
        for x, y in zip(v[0::2], v[1::2]):
            print(f'{x} {y} {height(x, y):.3f}')
        return
    nx = int(round((X1 - X0) / STEP)) + 1
    ny = int(round((Y1 - Y0) / STEP)) + 1
    pos, nrm = [], []
    for i in range(nx):
        for j in range(ny):
            x, y = X0 + i * STEP, Y0 + j * STEP
            z = height(x, y)
            d = 0.01
            dzdx = (height(x + d, y) - height(x - d, y)) / (2 * d)
            dzdy = (height(x, y + d) - height(x, y - d)) / (2 * d)
            n = (-dzdx, -dzdy, 1.0)
            k = math.sqrt(sum(c * c for c in n))
            pos.append(to_gltf(x, y, z))
            nrm.append(to_gltf(*(c / k for c in n)))
    idx = []
    for i in range(nx - 1):
        for j in range(ny - 1):
            a, b = i * ny + j, (i + 1) * ny + j
            # Counter-clockwise seen from above (+z), in Gazebo's frame.
            idx += [a, b, a + 1, a + 1, b, b + 1]
    pb = b''.join(struct.pack('<3f', *p) for p in pos)
    nb = b''.join(struct.pack('<3f', *n) for n in nrm)
    ib = b''.join(struct.pack('<I', k) for k in idx)
    blob = pb + nb + ib
    lo = [min(p[c] for p in pos) for c in range(3)]
    hi = [max(p[c] for p in pos) for c in range(3)]
    gltf = {
        'asset': {'version': '2.0', 'generator': 'gz_sonar make_seabed.py'},
        'scene': 0, 'scenes': [{'nodes': [0]}],
        'nodes': [{'name': 'seabed', 'mesh': 0}],
        'meshes': [{'name': 'seabed', 'primitives': [{
            'attributes': {'POSITION': 0, 'NORMAL': 1}, 'indices': 2, 'material': 0}]}],
        'materials': [{'name': 'sand', 'pbrMetallicRoughness': {
            'baseColorFactor': [0.62, 0.56, 0.42, 1.0], 'metallicFactor': 0.0,
            'roughnessFactor': 1.0}}],
        'buffers': [{'byteLength': len(blob)}],
        'bufferViews': [
            {'buffer': 0, 'byteOffset': 0, 'byteLength': len(pb), 'target': 34962},
            {'buffer': 0, 'byteOffset': len(pb), 'byteLength': len(nb), 'target': 34962},
            {'buffer': 0, 'byteOffset': len(pb) + len(nb), 'byteLength': len(ib),
             'target': 34963}],
        'accessors': [
            {'bufferView': 0, 'componentType': 5126, 'count': len(pos), 'type': 'VEC3',
             'min': lo, 'max': hi},
            {'bufferView': 1, 'componentType': 5126, 'count': len(nrm), 'type': 'VEC3'},
            {'bufferView': 2, 'componentType': 5125, 'count': len(idx), 'type': 'SCALAR'}],
    }
    js = json.dumps(gltf, separators=(',', ':')).encode()
    js += b' ' * (-len(js) % 4)
    blob += b'\0' * (-len(blob) % 4)
    out = pathlib.Path(__file__).resolve().parent / 'meshes' / 'seabed.glb'
    with open(out, 'wb') as f:
        f.write(struct.pack('<III', 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(blob)))
        f.write(struct.pack('<II', len(js), 0x4E4F534A) + js)
        f.write(struct.pack('<II', len(blob), 0x004E4942) + blob)
    print(f'{out}: {len(pos)} vertices, {len(idx) // 3} triangles')


if __name__ == '__main__':
    main()
