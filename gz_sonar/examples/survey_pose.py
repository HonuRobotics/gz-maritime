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


"""The vehicle's pose during a recorded survey, for the mapping examples."""

import math

import numpy as np


def rpy_matrix(roll, pitch, yaw):
    """Return the rotation matrix of fixed-axis roll, pitch, yaw (SDF's rpy)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def quat_matrix(w, x, y, z):
    """Return the rotation matrix of a unit quaternion."""
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Trajectory:
    """
    The recorded poses, interpolated to any time.

    poses: rows of (t, x, y, z, qw, qx, qy, qz) from record_survey.py. With
    noise, each queried pose is perturbed, to show what imperfect
    positioning does to a map: a seeded random offset of sigma_xy meters in
    x and y and sigma_yaw radians in heading, drawn afresh at every query,
    as a navigation solution's error would be from ping to ping.
    """

    def __init__(self, poses, sigma_xy=0.0, sigma_yaw=0.0, seed=1):
        p = np.asarray(poses)
        order = np.argsort(p[:, 0])
        self.p = p[order]
        self.sigma_xy = sigma_xy
        self.sigma_yaw = sigma_yaw
        self.rng = np.random.default_rng(seed)

    def at(self, t):
        """Return the position and rotation matrix at time t."""
        p = self.p
        pos = np.array([np.interp(t, p[:, 0], p[:, k]) for k in (1, 2, 3)])
        i = int(np.clip(np.searchsorted(p[:, 0], t), 1, len(p) - 1))
        a, b = p[i - 1, 4:8], p[i, 4:8]
        if np.dot(a, b) < 0:
            b = -b
        f = 0.0 if p[i, 0] == p[i - 1, 0] else (t - p[i - 1, 0]) / (p[i, 0] - p[i - 1, 0])
        q = (1 - f) * a + f * b
        q = q / np.linalg.norm(q)
        rot = quat_matrix(*q)
        if self.sigma_xy > 0 or self.sigma_yaw > 0:
            pos = pos + np.array([*self.rng.normal(0, self.sigma_xy, 2), 0.0])
            rot = rpy_matrix(0, 0, self.rng.normal(0, self.sigma_yaw)) @ rot
        return pos, rot
