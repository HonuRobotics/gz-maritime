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
Read a sonar's ROS 2 messages and print what each ping holds.

    read_sonar.py <topic> [--pings 3]

An image topic (marine_acoustic_msgs/ProjectedSonarImage, from an imaging
sonar or a sidescan side) is unpacked into a range-by-beam array: the
message stores the samples range-major (all beams at the first range, then
the next range), as uint8 for the imaging sonar and float32 dB for a
sidescan, and gives each beam a direction in the message's frame, which
puts Z along the beam and X up, so a beam's bearing is atan2(-y, z),
positive to port. For each ping it prints the frame, the size, the range
span and the brightest echo with its range and bearing.

A topic ending in `detections` (marine_acoustic_msgs/SonarDetections, from
a profiler) is read as soundings: each has a two-way travel time and an
across-track angle, rx_angles, positive to starboard. Range is half the
travel time times the sound speed in ping_info; for each ping it prints
the good soundings' angle, range, across-track distance and depth below
the sonar.
"""

import argparse
import math
import sys

from marine_acoustic_msgs.msg import ProjectedSonarImage, SonarDetections, SonarImageData
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def image_array(msg):
    """Return the image as a (samples, beams) array, range-major as stored."""
    dtype = {SonarImageData.DTYPE_UINT8: np.uint8,
             SonarImageData.DTYPE_FLOAT32: np.float32}[msg.image.dtype]
    beams = msg.image.beam_count
    data = np.frombuffer(bytes(msg.image.data), dtype=dtype)
    return data.reshape(len(msg.ranges), beams)


def describe_image(msg):
    """Return a line about one ProjectedSonarImage."""
    img = image_array(msg)
    k, j = np.unravel_index(np.argmax(img), img.shape)
    line = (f'frame {msg.header.frame_id}: {img.shape[1]} beams x {img.shape[0]} samples, '
            f'{msg.ranges[0]:.2f} to {msg.ranges[-1]:.2f} m, {img.dtype}; '
            f'brightest {img[k, j]:.1f} at {msg.ranges[k]:.2f} m')
    if img.shape[1] > 1:  # a sidescan side is one beam: no bearing to give
        d = msg.beam_directions[j]
        bearing = math.degrees(math.atan2(-d.y, d.z))
        line += (', on axis' if abs(bearing) < 0.05 else
                 f', {abs(bearing):.1f} deg to {"port" if bearing > 0 else "starboard"}')
    return line


def describe_detections(msg):
    """Return lines about one SonarDetections: its good soundings."""
    c = msg.ping_info.sound_speed
    good = [i for i, f in enumerate(msg.flags) if f.flag == 0]
    lines = [f'frame {msg.header.frame_id}: {len(msg.flags)} soundings, {len(good)} good']
    for i in good[:5]:
        a = msg.rx_angles[i]
        r = c * msg.two_way_travel_times[i] / 2.0
        lines.append(f'  {math.degrees(a):6.1f} deg (+ starboard): range {r:5.2f} m, '
                     f'across {r * math.sin(a):6.2f} m, depth {r * math.cos(a):5.2f} m')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[1])
    parser.add_argument('topic')
    parser.add_argument('--pings', type=int, default=3)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Node('read_sonar')
    detections = args.topic.endswith('detections')
    seen = []

    def on_message(msg):
        seen.append(msg)
        print(describe_detections(msg) if detections else describe_image(msg), flush=True)

    node.create_subscription(SonarDetections if detections else ProjectedSonarImage,
                             args.topic, on_message, qos_profile_sensor_data)
    try:
        while rclpy.ok() and len(seen) < args.pings:
            rclpy.spin_once(node, timeout_sec=1.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
