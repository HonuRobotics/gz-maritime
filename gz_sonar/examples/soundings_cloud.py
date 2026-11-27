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
Turn a recorded survey's profiler soundings into a georeferenced point cloud.

    soundings_cloud.py <survey.npz> <out.ply> [--mount 0.35 0 -0.08]
                       [--mount-rpy 0 1.5708 0] [--pose-noise 0]
                       [--heading-noise 0]

Each good sounding (flag 0) is a range, half the two-way travel time
times the sound speed the sonar assumes, along its angle in the
profiler's fan, placed in the world through the profiler's mount on the
vehicle and the vehicle's pose at the ping. --mount and --mount-rpy are
the profiler's beam frame on the vehicle (default: the BlueBoat's
Surveyor, looking down); --pose-noise (m) and --heading-noise (degrees)
perturb every ping's pose. Writes the points as an ASCII PLY, and a plan
view colored by height beside it (<out>.png).
"""

import argparse
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from survey_pose import rpy_matrix, Trajectory  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('survey')
    parser.add_argument('out')
    parser.add_argument('--mount', type=float, nargs=3, default=[0.35, 0.0, -0.08])
    parser.add_argument('--mount-rpy', type=float, nargs=3, default=[0.0, math.pi / 2, 0.0])
    parser.add_argument('--pose-noise', type=float, default=0.0)
    parser.add_argument('--heading-noise', type=float, default=0.0)
    args = parser.parse_args()

    d = np.load(args.survey)
    traj = Trajectory(d['pose'], args.pose_noise, math.radians(args.heading_noise))
    mount = np.array(args.mount)
    beam = rpy_matrix(*args.mount_rpy)
    points = []
    for t, c, angles, tts, flags in zip(d['p_t'], d['p_c'], d['p_angle'], d['p_tt'],
                                        d['p_flag']):
        good = (flags == 0) & np.isfinite(angles)
        if not good.any():
            continue
        r = 0.5 * c * tts[good]
        a = angles[good]
        local = np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], axis=1)
        pos, rot = traj.at(t)
        points.append(pos + (mount + local @ beam.T) @ rot.T)
    pts = np.concatenate(points)
    with open(args.out, 'w') as f:
        f.write(f'ply\nformat ascii 1.0\nelement vertex {len(pts)}\n'
                'property float x\nproperty float y\nproperty float z\nend_header\n')
        np.savetxt(f, pts, fmt='%.4f')

    fig, ax = plt.subplots(figsize=(9, 7))
    sc = ax.scatter(pts[:, 0], pts[:, 1], c=pts[:, 2], s=1, cmap='viridis',
                    vmin=np.percentile(pts[:, 2], 1), vmax=np.percentile(pts[:, 2], 99.5))
    ax.set_aspect('equal')
    ax.set_xlabel('x, east (m)')
    ax.set_ylabel('y, north (m)')
    fig.colorbar(sc, ax=ax, label='height (m)', shrink=0.8)
    ax.set_title(f'Profiler soundings, {len(pts)} points', fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out.rsplit('.', 1)[0] + '.png', dpi=120)
    print(f'{args.out}: {len(pts)} points')


if __name__ == '__main__':
    main()
