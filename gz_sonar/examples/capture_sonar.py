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

    capture_sonar.py <topic> <out_dir> [name] [pings] [timeout_s]

Writes gz_sonar_<name>.npy (an imaging sonar's 8-bit frame as float, or a
sidescan's dB), gz_sonar_<name>_u8.npy and gz_sonar_<name>.json, from the
last of the given number of pings.
"""

import json
import pathlib
import sys
import threading

from gz.maritime.sonar.sonar_image_pb2 import SonarImage
from gz.transport import Node
import numpy as np


def main():
    topic = sys.argv[1]
    out = pathlib.Path(sys.argv[2])
    name = sys.argv[3] if len(sys.argv) > 3 else 'capture'
    want = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    timeout = float(sys.argv[5]) if len(sys.argv) > 5 else 30.0
    out.mkdir(parents=True, exist_ok=True)

    got = []
    done = threading.Event()

    def on_ping(msg):
        got.append(msg)
        if len(got) >= want:
            done.set()

    node = Node()
    if not node.subscribe(SonarImage, topic, on_ping):
        sys.exit(f'cannot subscribe to {topic}')
    if not done.wait(timeout):
        sys.exit(f'{len(got)} of {want} pings on {topic} in {timeout} s')

    msg = got[want - 1]
    beams, samples = msg.beam_count, msg.sample_count
    stem = out / f'gz_sonar_{name}'
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
    stem.with_suffix('.json').write_text(json.dumps(meta))
    print(f'{stem}.npy: ping {msg.ping}, {beams} x {samples}')


if __name__ == '__main__':
    main()
