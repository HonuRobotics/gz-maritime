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
Build a georeferenced sidescan mosaic from a recorded survey.

    sidescan_mosaic.py <survey.npz> <out.png> [--resolution 0.25]
                       [--mount 0.2 0 -0.08] [--pose-noise 0] [--heading-noise 0]
                       [--min-speed 0.3] [--max-turn-rate 2]

Each line is placed by the vehicle's pose at its time: the slant range of
every sample becomes a ground range, sqrt(r^2 - h^2), with the altitude h
taken from the line's first seabed return (the edge of the nadir gap), and
the sample is laid along the ground perpendicular to the heading, to port
for a port line and to starboard for a starboard one. Cells that several
samples fall in keep their mean level.

The first return is tracked as a real sidescan's bottom track is: each
line smoothed over half a meter, its first sample within 20 dB of the
line's peak, then the median over 21 lines, so a speckle peak in the water
column does not pass for the seabed. It comes from where the beam's lower
edge first meets the seabed, a little beyond straight down, so h reads
slightly high and the ground ranges nearest the nadir gap slightly short.

Lines taken while the vehicle is
nearly still (--min-speed, m/s) or turning (--max-turn-rate, degrees/s)
are left out, as a survey's mosaic leaves out its turns: laid down
perpendicular to a heading that is changing, they fan out across the map.
This assumes a flat seabed, as a real sidescan mosaic does; objects
standing proud are laid down at their slant range, and their shadows show
where the seabed was hidden.

--mount is the sidescan's position on the vehicle (default: the
BlueBoat's); --pose-noise (m) and --heading-noise (degrees) perturb every
line's pose, to show what imperfect positioning does to the mosaic. Writes
the mosaic as a PNG, north up, and as <out>.npz (the grid in dB, its
origin and resolution).
"""

import argparse
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from survey_pose import Trajectory  # noqa: E402


def bottom_track(db, r, floor_db=20.0, window=0.5, lines=21):
    """
    Return each line's altitude: the range of its first seabed return.

    The first sample of the line, smoothed over `window` meters, within
    `floor_db` of its peak; then the median over `lines` neighboring lines.
    """
    n = max(1, int(round(window / (r[1] - r[0]))))
    padded = np.pad(db, ((0, 0), (n // 2, n - 1 - n // 2)), mode='edge')
    smooth = np.lib.stride_tricks.sliding_window_view(padded, n, axis=1).mean(-1)
    lit = smooth > smooth.max(1, keepdims=True) - floor_db
    h = np.where(lit.any(1), r[np.argmax(lit, axis=1)], np.nan)
    padded = np.pad(h, lines // 2, mode='edge')
    return np.nanmedian(np.lib.stride_tricks.sliding_window_view(padded, lines), axis=1)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('survey')
    parser.add_argument('out')
    parser.add_argument('--resolution', type=float, default=0.25)
    parser.add_argument('--mount', type=float, nargs=3, default=[0.2, 0.0, -0.08])
    parser.add_argument('--pose-noise', type=float, default=0.0)
    parser.add_argument('--heading-noise', type=float, default=0.0)
    parser.add_argument('--min-speed', type=float, default=0.3)
    parser.add_argument('--max-turn-rate', type=float, default=2.0)
    args = parser.parse_args()

    d = np.load(args.survey)
    traj = Trajectory(d['pose'], args.pose_noise, math.radians(args.heading_noise))
    truth = Trajectory(d['pose'])

    def steady(t):
        # Speed and turn rate over a tenth of a second, from the true track.
        (p0, r0), (p1, r1) = truth.at(t - 0.05), truth.at(t + 0.05)
        speed = np.linalg.norm((p1 - p0)[:2]) / 0.1
        turn = abs(math.atan2(r1[1, 0], r1[0, 0]) - math.atan2(r0[1, 0], r0[0, 0]))
        turn = min(turn, 2 * math.pi - turn) / 0.1
        return speed >= args.min_speed and turn <= math.radians(args.max_turn_rate)

    topics = [str(t) for t in d['sidescan_topics']]
    mount = np.array(args.mount)
    xs, ys, vs = [], [], []
    for k, topic in enumerate(topics):
        side = 1.0 if 'port' in topic else -1.0
        r = d[f's{k}_r'].astype(float)
        heights = bottom_track(d[f's{k}_db'], r)
        for t, db, h in zip(d[f's{k}_t'], d[f's{k}_db'], heights):
            if not steady(t):
                continue
            if not math.isfinite(h):
                continue
            pos, rot = traj.at(t)
            sensor = pos + rot @ mount
            out = rot @ np.array([0.0, side, 0.0])
            out[2] = 0.0
            out /= np.linalg.norm(out)
            keep = r > h
            g = np.sqrt(r[keep] ** 2 - h ** 2)
            xs.append(sensor[0] + g * out[0])
            ys.append(sensor[1] + g * out[1])
            vs.append(db[keep])
    x, y, v = np.concatenate(xs), np.concatenate(ys), np.concatenate(vs)
    res = args.resolution
    x0, y0 = math.floor(x.min()), math.floor(y.min())
    nx = int((x.max() - x0) / res) + 1
    ny = int((y.max() - y0) / res) + 1
    ix = ((x - x0) / res).astype(int)
    iy = ((y - y0) / res).astype(int)
    total = np.zeros((ny, nx))
    count = np.zeros((ny, nx))
    np.add.at(total, (iy, ix), v)
    np.add.at(count, (iy, ix), 1)
    grid = np.where(count > 0, total / np.maximum(count, 1), np.nan)
    np.savez(args.out.rsplit('.', 1)[0] + '.npz', db=grid, origin=[x0, y0], resolution=res)

    fig, ax = plt.subplots(figsize=(9, 9 * ny / nx + 0.8))
    ax.set_facecolor('white')  # cells no line reached
    ax.imshow(grid, origin='lower', cmap='copper', vmin=-60, vmax=0,
              extent=[x0, x0 + nx * res, y0, y0 + ny * res], interpolation='nearest')
    ax.set_xlabel('x, east (m)')
    ax.set_ylabel('y, north (m)')
    noise = (f', pose noise {args.pose_noise} m, heading noise {args.heading_noise} deg'
             if args.pose_noise or args.heading_noise else '')
    ax.set_title(f'Sidescan mosaic, dB re full scale{noise}', fontsize=10)
    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f'{args.out}: {nx} x {ny} cells of {res} m, {np.count_nonzero(count)} filled')


if __name__ == '__main__':
    main()
