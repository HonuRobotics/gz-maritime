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
Score a point cloud of a shipwreck survey against the wreck's ground truth.

    compare_wreck.py <points.ply> <wreck model dir> [--pose 0 0 -8 0 0 0.6]
                     [--seabed -8] [--margin 1.0]

The wreck's meshes are its exact ground-truth geometry: every part's .glb
shares the model frame, and the model's visuals turn each from glTF's
axes into Gazebo's (roll 90 degrees, then yaw 90 degrees). This reads
them, places them with the model's world pose (--pose, x y z roll pitch
yaw, as the world gives it), and measures each point's distance to the
nearest surface: a wreck triangle, or the flat seabed at --seabed. Points
over the wreck (within --margin of its footprint) are scored separately
from the open seabed. Prints the distances' median, 95th percentile and
maximum for each, and writes a histogram beside the cloud.
"""

import argparse
import glob
import json
import math
import os
import struct

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from survey_pose import rpy_matrix  # noqa: E402

COMPONENTS = {5121: np.uint8, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}


def read_glb(path):
    """Return the triangles of a .glb's meshes as an (n, 3, 3) array, glTF axes."""
    with open(path, 'rb') as f:
        data = f.read()
    json_len = struct.unpack('<I', data[12:16])[0]
    gltf = json.loads(data[20:20 + json_len])
    binary = data[20 + json_len + 8:]

    def accessor(i):
        a = gltf['accessors'][i]
        view = gltf['bufferViews'][a['bufferView']]
        width = {'SCALAR': 1, 'VEC3': 3}[a['type']]
        start = view.get('byteOffset', 0) + a.get('byteOffset', 0)
        arr = np.frombuffer(binary, COMPONENTS[a['componentType']], a['count'] * width, start)
        return arr.reshape(-1, width) if width > 1 else arr

    tris = []
    for mesh in gltf['meshes']:
        for prim in mesh['primitives']:
            pos = accessor(prim['attributes']['POSITION']).astype(float)
            idx = accessor(prim['indices']).astype(int)
            tris.append(pos[idx.reshape(-1, 3)])
    return np.concatenate(tris)


def point_triangle_distance(p, tri):
    """Return the distance from each point in p (n, 3) to the nearest of tri (m, 3, 3)."""
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    best = np.full(len(p), np.inf)
    ab, ac = b - a, c - a
    for k in range(0, len(p), 256):
        q = p[k:k + 256, None, :]
        ap = q - a
        d1, d2 = (ab * ap).sum(-1), (ac * ap).sum(-1)
        bp = q - b
        d3, d4 = (ab * bp).sum(-1), (ac * bp).sum(-1)
        cp = q - c
        d5, d6 = (ab * cp).sum(-1), (ac * cp).sum(-1)
        va = d3 * d6 - d5 * d4
        vb = d5 * d2 - d1 * d6
        vc = d1 * d4 - d3 * d2
        denom = va + vb + vc
        denom = np.where(np.abs(denom) < 1e-12, 1e-12, denom)
        v = vb / denom
        w = vc / denom
        # Interior projection, then clamp to the closest edge or vertex.
        closest = a + v[..., None] * ab + w[..., None] * ac
        cases = [
            ((d1 <= 0) & (d2 <= 0), a + 0 * q),
            ((d3 >= 0) & (d4 <= d3), b + 0 * q),
            ((d6 >= 0) & (d5 <= d6), c + 0 * q),
        ]
        e = np.clip(d1 / np.where(d1 - d3 == 0, 1e-12, d1 - d3), 0, 1)
        cases.append(((vc <= 0) & (d1 >= 0) & (d3 <= 0), a + e[..., None] * ab))
        e = np.clip(d2 / np.where(d2 - d6 == 0, 1e-12, d2 - d6), 0, 1)
        cases.append(((vb <= 0) & (d2 >= 0) & (d6 <= 0), a + e[..., None] * ac))
        num = d4 - d3
        e = np.clip(num / np.where(num + d5 - d6 == 0, 1e-12, num + d5 - d6), 0, 1)
        cases.append(((va <= 0) & (num >= 0) & (d5 - d6 >= 0), b + e[..., None] * (c - b)))
        for mask, pt in reversed(cases):
            closest = np.where(mask[..., None], pt, closest)
        dist = np.linalg.norm(q - closest, axis=-1).min(axis=1)
        best[k:k + 256] = dist
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('points')
    parser.add_argument('model_dir')
    parser.add_argument('--pose', type=float, nargs=6, default=[0, 0, -8, 0, 0, 0.6])
    parser.add_argument('--seabed', type=float, default=-8.0)
    parser.add_argument('--margin', type=float, default=1.0)
    args = parser.parse_args()

    with open(args.points) as f:
        header = 0
        for line in f:
            header += 1
            if line.strip() == 'end_header':
                break
    pts = np.loadtxt(args.points, skiprows=header)

    to_gazebo = rpy_matrix(math.pi / 2, 0, math.pi / 2)
    place = rpy_matrix(*args.pose[3:])
    tris = np.concatenate([read_glb(m) for m in sorted(glob.glob(
        os.path.join(args.model_dir, 'meshes', '*.glb')))])
    tris = tris @ to_gazebo.T @ place.T + np.array(args.pose[:3])
    # Only the wreck above the seabed can be seen; buried geometry is not scored.
    lo = tris.reshape(-1, 3).min(0) - args.margin
    hi = tris.reshape(-1, 3).max(0) + args.margin
    over = np.all((pts[:, :2] >= lo[:2]) & (pts[:, :2] <= hi[:2]), axis=1)
    d_seabed = np.abs(pts[:, 2] - args.seabed)
    d = d_seabed.copy()
    d[over] = np.minimum(d_seabed[over], point_triangle_distance(pts[over], tris))

    def report(name, x):
        if len(x):
            print(f'{name}: {len(x)} points, distance to the true surface: median '
                  f'{np.median(x):.3f} m, 95 % {np.percentile(x, 95):.3f} m, '
                  f'max {x.max():.3f} m')
    report('over the wreck', d[over])
    report('open seabed', d[~over])
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.hist([d[over], d[~over]], bins=60, range=(0, max(0.5, np.percentile(d, 99.5))),
            label=['over the wreck', 'open seabed'], stacked=True)
    ax.set_xlabel('distance from each sounding to the true surface (m)')
    ax.set_ylabel('soundings')
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.points.rsplit('.', 1)[0] + '_error.png', dpi=110)


if __name__ == '__main__':
    main()
