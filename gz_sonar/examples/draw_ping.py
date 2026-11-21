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

Also draws what capture_sonar.py saved from a Gazebo sonar, and, where a
capture has ground-truth labels, the fan beside its labels
(gz_sonar_<name>_labels.png).
"""

import json
import pathlib
import sys
import warnings

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


def draw_labels(stem):
    """Draw the fan beside its labels, one color per object, seabed gray."""
    meta = json.loads(stem.with_suffix('.json').read_text())
    frame = np.load(str(stem) + '_u8.npy').astype(float)
    ids = np.load(str(stem) + '_labels.npy')
    names = meta['label_names']
    az = np.array(meta['azimuths'])
    r = np.array(meta['ranges'])
    A, R = np.meshgrid(az, r, indexing='ij')
    x = R * np.cos(A)
    y = R * np.sin(A)
    # One color per label: black for nothing, gray for the seabed (the
    # label named in the world's <seabed>, here any name with "seabed"),
    # a distinct color for each object.
    palette = plt.get_cmap('tab10').colors
    colors = np.zeros((len(names), 3))
    handles = []
    k = 0
    for i, n in enumerate(names):
        if i == 0:
            continue
        short = n.split('::')[-1]
        if 'seabed' in short or 'ground' in short:
            colors[i] = (0.35, 0.35, 0.35)
        else:
            colors[i] = palette[k % len(palette)]
            k += 1
        if (ids == i).any():
            handles.append(plt.Line2D([], [], marker='s', linestyle='',
                                      color=colors[i], label=short))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
    a1.pcolormesh(-y, x, frame, cmap='copper', vmin=0, vmax=255,
                  shading='gouraud')
    a1.set_title('sonar image')
    # One masked layer per label, each in its own color.
    # Cell centers on a polar grid are not monotonic in x and y; matplotlib
    # warns but draws each cell where it belongs.
    warnings.filterwarnings('ignore', message='The input coordinates to pcolormesh')
    a2.pcolormesh(-y, x, np.zeros_like(frame), cmap='gray', vmin=0, vmax=1,
                  shading='nearest')
    for i in np.unique(ids):
        if i == 0:
            continue
        layer = np.ma.masked_where(ids != i, np.ones_like(frame))
        cmap = matplotlib.colors.ListedColormap([colors[i]])
        a2.pcolormesh(-y, x, layer, cmap=cmap, vmin=0, vmax=1,
                      shading='nearest')
    a2.set_title('labels: the object each echo came from')
    a2.legend(handles=handles, loc='upper right', fontsize=8, framealpha=0.8)
    for a in (a1, a2):
        a.set_aspect('equal')
        a.set_facecolor('black')
        a.set_xlabel('starboard (m)')
    a1.set_ylabel('forward (m)')
    fig.tight_layout()
    fig.savefig(str(stem) + '_labels.png', dpi=120)
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
            if pathlib.Path(str(stem) + '_labels.npy').exists():
                draw_labels(stem)
                print(f'{stem}_labels.png')
        elif kind == 'sidescan':
            draw_sidescan(stem)
        print(f'{stem}.png')
    csv = out / 'gz_sonar_profiling.csv'
    if csv.exists():
        draw_soundings(csv)
        print(csv.with_suffix('.png'))


if __name__ == '__main__':
    main()
