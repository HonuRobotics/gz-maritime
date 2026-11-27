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
Record a survey: sidescan lines, profiler soundings and the vehicle's pose.

    record_survey.py <world> <out.npz> <seconds> [--model blueboat]
                     [--sidescan TOPIC ...] [--profiler TOPIC]

Saves, with the simulation time of each: every sidescan line (levels in
dB, the time-varied gain and gain applied, and the ranges), every ping of
soundings (angles, two-way travel times, flags, sound speed), and the
vehicle's pose (position and orientation quaternion w, x, y, z) from
/world/<world>/dynamic_pose/info. It records for the given seconds, or
until interrupted (Ctrl-C), and then saves. sidescan_mosaic.py and
soundings_cloud.py read the file.
"""

import argparse
import os
import threading
import time

from gz.maritime.sonar.sonar_detections_pb2 import SonarDetections
from gz.maritime.sonar.sonar_image_pb2 import SonarImage
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node
import numpy as np


def stamp(h):
    """Return a header's time in seconds."""
    return h.stamp.sec + 1e-9 * h.stamp.nsec


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('world')
    parser.add_argument('out')
    parser.add_argument('seconds', type=float)
    parser.add_argument('--model', default='blueboat')
    parser.add_argument('--sidescan', nargs='*', default=[
        '/blueboat/sidescan/port/image', '/blueboat/sidescan/stbd/image'])
    parser.add_argument('--profiler', default='/blueboat/profiler/detections')
    args = parser.parse_args()

    lock = threading.Lock()
    lines = {t: [] for t in args.sidescan}
    ranges = {}
    pings = []
    poses = []

    def on_line(topic):
        def cb(m):
            with lock:
                lines[topic].append((stamp(m.header),
                                     np.frombuffer(m.data, dtype='<f4').copy(),
                                     np.array(m.tvg, dtype=np.float32), m.gain))
                ranges[topic] = np.array(m.ranges, dtype=np.float32)
        return cb

    def on_soundings(m):
        with lock:
            pings.append((stamp(m.header), m.sound_speed, list(m.angles),
                          list(m.travel_times), list(m.flags)))

    def on_pose(m):
        for p in m.pose:
            if p.name == args.model:
                q = p.orientation
                with lock:
                    poses.append((stamp(m.header), p.position.x, p.position.y,
                                  p.position.z, q.w, q.x, q.y, q.z))

    node = Node()
    for t in args.sidescan:
        node.subscribe(SonarImage, t, on_line(t))
    if args.profiler:
        node.subscribe(SonarDetections, args.profiler, on_soundings)
    node.subscribe(Pose_V, f'/world/{args.world}/dynamic_pose/info', on_pose)
    try:
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        pass

    with lock:
        out = {'pose': np.array(poses), 'sidescan_topics': np.array(args.sidescan)}
        for k, t in enumerate(args.sidescan):
            ls = lines[t]
            out[f's{k}_t'] = np.array([v[0] for v in ls])
            out[f's{k}_db'] = np.array([v[1] for v in ls])
            out[f's{k}_tvg'] = np.array([v[2] for v in ls])
            out[f's{k}_gain'] = np.array([v[3] for v in ls])
            out[f's{k}_r'] = ranges.get(t, np.zeros(0, np.float32))
        n = max((len(p[2]) for p in pings), default=0)

        def pad(v):
            return list(v) + [np.nan] * (n - len(v))
        out['p_t'] = np.array([p[0] for p in pings])
        out['p_c'] = np.array([p[1] for p in pings])
        out['p_angle'] = np.array([pad(p[2]) for p in pings])
        out['p_tt'] = np.array([pad(p[3]) for p in pings])
        out['p_flag'] = np.array([pad([float(f) for f in p[4]]) for p in pings])
    np.savez(args.out, **out)
    print(f'{args.out}: {len(out["pose"])} poses, '
          + ', '.join(f'{len(lines[t])} lines on {t}' for t in args.sidescan)
          + f', {len(pings)} pings of soundings', flush=True)
    # Skip interpreter teardown with a live transport node.
    os._exit(0)


if __name__ == '__main__':
    main()
