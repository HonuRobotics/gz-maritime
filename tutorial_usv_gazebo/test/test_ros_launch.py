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
End-to-end test of the launch files: the ROS side of a two-boat bring-up.

Complements test_gz_launch.py, which pins the model and world by running gz
directly: this one runs two_usvs.launch.xml headless, i.e. the simulation
part once and the spawn part twice, with each boat's bridge and
robot_state_publisher in its own namespace and TF prefix, and the clock
bridged once.
"""

import functools
import os
import signal
import uuid

from conftest import launch_sim, make_cli, poll_until, stop_process_group
import pytest

LAUNCH = ['ros2', 'launch', 'tutorial_usv_gazebo', 'two_usvs.launch.xml',
          'gazebo_gui:=false']
BOATS = ('boat_a', 'boat_b')

ros = make_cli('ros2', default_timeout=15)


@pytest.fixture(scope='module')
def sim(request):
    """Bring up two_usvs.launch.xml headless on isolated domains; yield the env."""
    env = dict(os.environ,
               GZ_PARTITION=f'test_{uuid.uuid4().hex[:8]}',
               # uuid, not pid: parallel colcon test runs could share a domain
               # when two pids agree modulo 100.
               ROS_DOMAIN_ID=str(int(uuid.uuid4().hex[:2], 16) % 100 + 1))
    # SIGINT first so ros2 launch shuts its children down in order.
    return launch_sim(
        request, 'ros2 launch', LAUNCH, env,
        ready=lambda e: all(f'/{n}/robot_state_publisher' in ros(e, 'node', 'list')[1]
                            for n in BOATS),
        stop=functools.partial(stop_process_group,
                               sig=signal.SIGINT, grace=20))


def test_container_and_per_boat_nodes_up(sim):
    """One container; a bridge and a state publisher per boat, in its namespace."""
    needed = ['/ros_gz_container', '/ros_gz_bridge']
    needed += [f'/{n}/{node}' for n in BOATS
               for node in ('ros_gz_bridge', 'robot_state_publisher')]
    poll_until(
        lambda: all(n in ros(sim, 'node', 'list')[1] for n in needed), 30,
        lambda: f'missing nodes; last listing:\n{ros(sim, "node", "list")[1]}')


def test_clock_bridged_once(sim):
    """/clock has exactly one publisher: the simulation launch, not the boats."""
    code, out, err = ros(sim, 'topic', 'echo', '/clock', '--once', timeout=30)
    assert code == 0 and 'clock' in out, f'no /clock over the bridge\n{err}'
    code, out, _ = ros(sim, 'topic', 'info', '/clock', timeout=30)
    assert code == 0 and 'Publisher count: 1' in out, out


def test_each_boat_bridged_under_its_name(sim):
    """Every entry of each boat's bridge config is on the ROS graph, prefixed."""
    needed = []
    for name in BOATS:
        needed += [f'/{name}/motor_{side}/thrust{suffix}'
                   for side in ('port', 'stbd') for suffix in ('', '/ang_vel')]
        needed += [f'/{name}/{topic}'
                   for topic in ('joint_states', 'imu', 'navsat', 'magnetometer')]
    poll_until(
        lambda: all(t in ros(sim, 'topic', 'list')[1].split() for t in needed), 60,
        lambda: f'topics not bridged; last listing:\n'
                f'{ros(sim, "topic", "list")[1]}')


def test_imu_messages_arrive_in_prefixed_frames(sim):
    """An IMU sample per boat crosses the bridge, stamped <name>/imu_link."""
    for name in BOATS:
        code, out, err = ros(sim, 'topic', 'echo', f'/{name}/imu', '--once',
                             timeout=60)
        assert code == 0 and 'angular_velocity' in out, f'no IMU message for {name}\n{err}'
        assert f'frame_id: {name}/imu_link' in out, f'unexpected IMU frame:\n{out}'


def test_tf_carries_both_prefixes(sim):
    """Each robot_state_publisher publishes its boat's frames under its prefix."""
    code, out, err = ros(sim, 'topic', 'echo', '/tf_static', '--once',
                         '--qos-durability', 'transient_local',
                         '--qos-reliability', 'reliable', timeout=60)
    assert code == 0, f'no /tf_static\n{err}'
    # Two latched publishers: --once shows one of them; the other must be
    # there too. Read /tf, which both publish continuously from joint states.
    seen = set()

    def both_seen():
        code, out, _ = ros(sim, 'topic', 'echo', '/tf', '--once', timeout=30)
        if code == 0:
            for name in BOATS:
                if f'{name}/motor_' in out:
                    seen.add(name)
        return seen == set(BOATS)

    poll_until(both_seen, 60,
               lambda: f'propeller transforms seen only for {sorted(seen)}')
