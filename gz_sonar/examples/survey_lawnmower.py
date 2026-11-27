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
Drive a two-propeller boat along survey lines, a lawnmower pattern.

    survey_lawnmower.py <world> [--model blueboat] [--x0 -40] [--x1 40]
                        [--y0 -30] [--y1 30] [--spacing 10] [--thrust 0.3]

Runs lines parallel to x, from x0 to x1 and back, starting at y0 and
stepping by spacing up to y1. Each line is held by a heading and
cross-track controller on the two propellers' normalized commands
(/<model>/prop_port/cmd and /<model>/prop_stbd/cmd on gz-transport); the
boat turns at the end of each line on the same controller. It reads the
boat's pose from /world/<world>/dynamic_pose/info, stops the propellers at
the end, and prints each line as it starts.
"""

import argparse
import math
import os
import threading
import time

from gz.msgs.double_pb2 import Double
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node


def wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('world')
    parser.add_argument('--model', default='blueboat')
    parser.add_argument('--x0', type=float, default=-40.0)
    parser.add_argument('--x1', type=float, default=40.0)
    parser.add_argument('--y0', type=float, default=-30.0)
    parser.add_argument('--y1', type=float, default=30.0)
    parser.add_argument('--spacing', type=float, default=10.0)
    parser.add_argument('--thrust', type=float, default=0.3)
    args = parser.parse_args()

    lock = threading.Lock()
    state = {}

    def on_pose(msg):
        for p in msg.pose:
            if p.name == args.model:
                q = p.orientation
                with lock:
                    state['x'], state['y'] = p.position.x, p.position.y
                    state['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                              1 - 2 * (q.y * q.y + q.z * q.z))

    node = Node()
    node.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', on_pose)
    port = node.advertise(f'/{args.model}/prop_port/cmd', Double)
    stbd = node.advertise(f'/{args.model}/prop_stbd/cmd', Double)

    def send(a, b):
        for pub, v in ((port, a), (stbd, b)):
            m = Double()
            m.data = max(-1.0, min(1.0, v))
            pub.publish(m)

    lines = []
    y, forward = args.y0, True
    while y <= args.y1 + 1e-6:
        lines.append((args.x0, args.x1, y) if forward else (args.x1, args.x0, y))
        y += args.spacing
        forward = not forward

    while 'x' not in state:
        time.sleep(0.1)
    for i, (xa, xb, yl) in enumerate(lines):
        heading = 0.0 if xb > xa else math.pi
        print(f'line {i + 1}/{len(lines)}: y = {yl:.1f}, x {xa:.0f} to {xb:.0f}', flush=True)
        while True:
            with lock:
                x, yb, yaw = state['x'], state['y'], state['yaw']
            if (x - xb) * math.cos(heading) >= 0.0:
                break
            # Cross-track error, positive to the left of the line's heading.
            cross = (yb - yl) * math.cos(heading)
            want = wrap(heading - max(-0.6, min(0.6, 0.25 * cross)))
            err = wrap(yaw - want)
            # Off heading by much (a turn), slow down and steer hard.
            base = args.thrust * (0.4 if abs(err) > 0.5 else 1.0)
            u = max(-0.25, min(0.25, 0.8 * err))
            send(base + u, base - u)
            time.sleep(0.05)
    send(0.0, 0.0)
    time.sleep(0.2)
    print('survey done', flush=True)
    os._exit(0)


if __name__ == '__main__':
    main()
