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
Save pings a Gazebo sonar publishes, in draw_ping.py's format.

    capture_sonar.py <topic> <out_dir> [--name N] [--count K | --ping P]
                     [--labels] [--timeout S]

Writes gz_sonar_<name>.npy (an imaging sonar's 8-bit frame as float, or a
sidescan's dB), gz_sonar_<name>_u8.npy and gz_sonar_<name>.json. --count K
(default 1) keeps the K-th ping received; --ping P waits for ping number P,
which is the same ping in every run with the same seed. --labels also saves
the sonar's ground truth (its <topic>/labels, on when the sensor sets
<labels>true</labels>) as gz_sonar_<name>_labels.npy, one label id per beam
and sample, with the name of each id in the .json.
"""

import argparse
import json
import os
import pathlib
import sys
import threading

from gz.maritime.sonar.sonar_image_pb2 import SonarImage
from gz.maritime.sonar.sonar_labels_pb2 import SonarLabels
from gz.transport import Node
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('topic')
    parser.add_argument('out_dir')
    parser.add_argument('--name', default='capture')
    which = parser.add_mutually_exclusive_group()
    which.add_argument('--count', type=int, default=1)
    which.add_argument('--ping', type=int)
    parser.add_argument('--labels', action='store_true')
    parser.add_argument('--timeout', type=float, default=60.0)
    args = parser.parse_args()
    out = pathlib.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    lock = threading.Lock()
    done = threading.Event()
    images = []
    labels = {}

    def wanted(ping, received):
        return ping == args.ping if args.ping is not None else \
            received == args.count

    def on_image(msg):
        with lock:
            images.append(msg)
            if wanted(msg.ping, len(images)) and \
                    (not args.labels or msg.ping in labels):
                done.set()

    def on_labels(msg):
        with lock:
            labels[msg.ping] = msg
            if any(wanted(m.ping, i + 1) and m.ping in labels
                   for i, m in enumerate(images)):
                done.set()

    node = Node()
    if not node.subscribe(SonarImage, args.topic, on_image):
        sys.exit(f'cannot subscribe to {args.topic}')
    if args.labels and not node.subscribe(SonarLabels, args.topic + '/labels',
                                          on_labels):
        sys.exit(f'cannot subscribe to {args.topic}/labels')
    if not done.wait(args.timeout):
        print(f'no wanted ping on {args.topic} in {args.timeout} s '
              f'({len(images)} received)', file=sys.stderr, flush=True)
        os._exit(1)

    with lock:
        msg = next(m for i, m in enumerate(images) if wanted(m.ping, i + 1))
        lab = labels.get(msg.ping)
    beams, samples = msg.beam_count, msg.sample_count
    stem = out / f'gz_sonar_{args.name}'
    if msg.data_type == SonarImage.UINT8:
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(beams, samples)
        np.save(str(stem) + '_u8.npy', frame)
        np.save(str(stem) + '.npy', frame.astype(np.float32))
        kind = 'imaging'
    else:
        db = np.frombuffer(msg.data, dtype='<f4').reshape(beams, samples)
        np.save(str(stem) + '.npy', db)
        kind = 'sidescan_line'
    meta = {
        'kind': kind,
        'ranges': list(msg.ranges),
        'azimuths': list(msg.azimuths),
        'ping': msg.ping,
        'stamp': msg.header.stamp.sec + 1e-9 * msg.header.stamp.nsec,
    }
    if lab is not None:
        ids = np.frombuffer(lab.labels, dtype=np.uint8).reshape(
            lab.beam_count, lab.sample_count)
        np.save(str(stem) + '_labels.npy', ids)
        meta['label_names'] = list(lab.names)
    stem.with_suffix('.json').write_text(json.dumps(meta))
    print(f'{stem}.npy: ping {msg.ping}, {beams} x {samples}'
          + (', with labels' if lab is not None else ''), flush=True)
    # Skip interpreter teardown with a live transport node.
    os._exit(0)


if __name__ == '__main__':
    main()
