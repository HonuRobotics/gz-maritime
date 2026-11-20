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
Draw what gz_sonar_render_ping wrote, as PNGs next to the data.

Imaging fans, a sidescan waterfall and the profiler's soundings:

    draw_ping.py <out_dir>
"""

import json
import pathlib
import sys

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def draw_fan(stem):
    meta = json.loads(stem.with_suffix('.json').read_text())
    frame = np.load(str(stem) + '_u8.npy').astype(float)
    az = np.array(meta['azimuths'])
    r = np.array(meta['ranges'])
    # Beams across, samples out: x forward, y to port (left).
    A, R = np.meshgrid(az, r, indexing='ij')
    x = R * np.cos(A)
    y = R * np.sin(A)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    # Polar cells: Gouraud shading needs no cell edges.
    ax.pcolormesh(-y, x, frame, cmap='copper', vmin=0, vmax=255,
                  shading='gouraud')
    ax.set_aspect('equal')
    ax.set_facecolor('black')
    ax.set_xlabel('starboard (m)')
    ax.set_ylabel('forward (m)')
    ax.set_title(f'{stem.name}: imaging sonar, 8-bit readout')
    fig.tight_layout()
    fig.savefig(str(stem) + '.png', dpi=120)
    plt.close(fig)


def draw_sidescan(stem):
    meta = json.loads(stem.with_suffix('.json').read_text())
    db = np.load(str(stem) + '.npy')
    r = np.array(meta['ranges'])
    along = np.array(meta['along_track'])
    fig, ax = plt.subplots(figsize=(7, 4.5))
    top = np.percentile(db, 99.5)
    ax.pcolormesh(r, along, db, cmap='copper', vmin=top - 40, vmax=top,
                  shading='auto')
    ax.set_xlabel('slant range to port (m)')
    ax.set_ylabel('along track (m)')
    ax.set_title('sidescan waterfall, dB with time-varied gain')
    fig.tight_layout()
    fig.savefig(str(stem) + '.png', dpi=120)
    plt.close(fig)


def draw_soundings(path):
    data = np.genfromtxt(path, delimiter=',', names=True)
    ok = data['flag'] == 0
    fig, ax = plt.subplots(figsize=(7, 4.5))
    sc = ax.scatter(data['x'][ok], data['y'][ok], c=data['z'][ok], s=4,
                    cmap='viridis')
    fig.colorbar(sc, ax=ax, label='height (m)')
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('profiling soundings, georeferenced with true poses')
    fig.tight_layout()
    fig.savefig(path.with_suffix('.png'), dpi=120)
    plt.close(fig)


def main():
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else '.')
    for meta in sorted(out.glob('gz_sonar_*.json')):
        kind = json.loads(meta.read_text())['kind']
        stem = meta.with_suffix('')
        if kind == 'imaging':
            draw_fan(stem)
        elif kind == 'sidescan':
            draw_sidescan(stem)
        print(f'{stem}.png')
    csv = out / 'gz_sonar_profiling.csv'
    if csv.exists():
        draw_soundings(csv)
        print(csv.with_suffix('.png'))


if __name__ == '__main__':
    main()
