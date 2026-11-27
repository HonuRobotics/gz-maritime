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
Drive a custom USV to the pinger it hears: the acoustic demo's controller.

Run in a vehicle's namespace. It reads the boat's acoustic receiver
(acoustic_receiver, ros_gz_interfaces/msg/ParamVec, one message per ping
heard) and its IMU, and drives the two thrusters (motor_port/cmd and
motor_stbd/cmd, a fraction of full thrust in [-1, 1]).

A ping comes once a second, and the boat turns in between, so a bearing is
turned into a heading the moment it arrives, with the IMU's yaw then, and
the boat steers to that heading until the next ping. A ping marked invalid
is ignored, and the median of the last three headings throws out a lone
outlier. Close to the pinger the boat stops and holds; with no ping for
silence_timeout seconds, the pinger switched off, it stops and waits for
one.

Close is loud: the receiver gives no range, and its amplitude (the
spreading loss, 20 dB down for each ten times the distance) is noisy, 1 dB
on the custom USV. So the boat stops on the median of the last three
amplitudes, once three have come, not on one ping: louder than
stop_amplitude, -16 dB, which is 6.3 m without noise and between about 5.5
and 7 m with it.

    ros2 run kai_custom_vehicle pinger_homing.py --ros-args -r __ns:=/custom_usv
"""

import math

from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import ParamVec
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


def yaw_of(q: Quaternion) -> float:
    """Return the yaw of a quaternion, rad."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle: float) -> float:
    """Wrap an angle into [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def circular_median(angles: list[float]) -> float:
    """Return the angle of a short list closest to all the others."""
    return min(angles, key=lambda a: sum(abs(wrap(a - b)) for b in angles))


def reading(msg: ParamVec) -> dict:
    """Return a receiver message's parameters by name."""
    out = {}
    for p in msg.params:
        v = p.value
        out[p.name] = v.bool_value if v.type == 1 else v.double_value
    return out


class PingerHoming(Node):
    """Steer to the heading of the last pings, stop when close or when it falls silent."""

    def __init__(self):
        super().__init__('pinger_homing')
        self.thrust = self.declare_parameter('thrust', 0.6).value
        self.turn_gain = self.declare_parameter('turn_gain', 1.5).value
        self.stop_amplitude = self.declare_parameter('stop_amplitude', -16.0).value
        self.silence_timeout = self.declare_parameter('silence_timeout', 3.0).value

        self.yaw = None
        self.headings = []
        self.amplitudes = []
        self.last_ping = None
        self.state = 'waiting'

        self.port = self.create_publisher(Float64, 'motor_port/cmd', 10)
        self.stbd = self.create_publisher(Float64, 'motor_stbd/cmd', 10)
        self.create_subscription(Imu, 'imu', self.on_imu, 10)
        self.create_subscription(ParamVec, 'acoustic_receiver', self.on_ping, 10)
        self.create_timer(0.1, self.on_timer)

    def on_imu(self, msg: Imu):
        self.yaw = yaw_of(msg.orientation)

    def on_ping(self, msg: ParamVec):
        r = reading(msg)
        if not r.get('valid', False) or self.yaw is None or 'bearing' not in r:
            return
        self.headings = (self.headings + [wrap(self.yaw + r['bearing'])])[-3:]
        if 'amplitude' in r:
            self.amplitudes = (self.amplitudes + [r['amplitude']])[-3:]
        self.last_ping = self.get_clock().now()
        self.get_logger().info(
            f"ping {r.get('frequency', 0) / 1000:.1f} kHz bearing "
            f"{math.degrees(r['bearing']):+.0f} deg, "
            f"{r.get('amplitude', float('nan')):.1f} dB")

    def loud(self) -> bool:
        """Return whether the last three pings say the pinger is close."""
        # The median of a full window of three, so one loud draw of the
        # noise does not stop the boat short, nor one quiet draw send it off
        # again; with fewer, the first ping alone would decide.
        return (len(self.amplitudes) == 3 and
                sorted(self.amplitudes)[1] >= self.stop_amplitude)

    def drive(self, port: float, stbd: float):
        self.port.publish(Float64(data=max(-1.0, min(1.0, port))))
        self.stbd.publish(Float64(data=max(-1.0, min(1.0, stbd))))

    def set_state(self, state: str):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state

    def on_timer(self):
        now = self.get_clock().now()
        silent = (self.last_ping is None or
                  (now - self.last_ping).nanoseconds * 1e-9 > self.silence_timeout)
        if silent:
            self.headings = []
            self.amplitudes = []
            self.set_state('waiting')
            self.drive(0.0, 0.0)
            return
        if self.loud():
            self.set_state('arrived')
            self.drive(0.0, 0.0)
            return
        self.set_state('homing')
        error = wrap(circular_median(self.headings) - self.yaw)
        turn = self.turn_gain * error
        # Slow down to turn hard: full thrust only when roughly on course.
        forward = self.thrust * max(0.0, math.cos(error))
        # A bearing to port, positive, turns the boat left: more starboard.
        self.drive(forward - turn, forward + turn)


def main():
    rclpy.init()
    node = PingerHoming()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
