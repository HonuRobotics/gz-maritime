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
The multi vehicle demo, end to end: one ocean, one vehicle of each type.

multi_vehicle_demo.launch.py brings the simulation up with a BlueBoat, a
BlueROV2 and an X500 on the landing pad. Each vehicle sits where it belongs
(the boat at the waterline, the ROV under it, the quad on the pad), each has
its own nodes, topics and TF prefix under its name, the clock has one
publisher, and a command to the boat moves the boat and nothing else.
Behaviour is measured in sim time, so a slow runner changes how long the
tests wait, never what they assert.

The vehicle packages have no rosdep keys, so a workspace without them skips
this module rather than failing it.
"""

import functools
import math
import os
import re
import signal
import subprocess
import tempfile
import uuid

from ament_index_python.packages import (get_package_share_directory,
                                         PackageNotFoundError)
from conftest import launch_sim, make_cli, poll_until, stop_process_group
import pytest

VEHICLE_PACKAGES = ('blueboat_gazebo', 'bluerov2_gazebo', 'x500_gazebo')
try:
    for package in VEHICLE_PACKAGES:
        get_package_share_directory(package)
except PackageNotFoundError as missing:
    pytest.skip(f'the demo needs every vehicle package: {missing}', allow_module_level=True)

WORLD_NAME = 'default'
VEHICLES = {
    # name: (expected z after settling, tolerance, topics under the name)
    'blueboat': (0.0, 0.1, ('motor_port/cmd', 'motor_stbd/cmd', 'ping/range', 'joint_states')),
    'bluerov2': (-1.0, 0.6, ('thruster_1/cmd', 'camera/image', 'joint_states')),
    'x500': (1.1, 0.2, ('imu', 'air_pressure', 'gps/fix', 'joint_states')),
}
_NUM = r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?'
POSE_TRIPLE = re.compile(rf'({_NUM}) ({_NUM}) ({_NUM})')

gz = make_cli('gz')
ros = make_cli('ros2', default_timeout=20)
stop = functools.partial(stop_process_group, sig=signal.SIGINT, grace=20)


def nodes(env):
    """Return the ROS node names."""
    return ros(env, 'node', 'list')[1].split()


def model_pose(env, name):
    """Return a model's world pose from `gz model -p`: (x, y, z, roll, pitch, yaw)."""
    code, out, err = gz(env, 'model', '-m', name, '-p', timeout=15)
    triples = POSE_TRIPLE.findall(out)
    assert code == 0 and len(triples) >= 2, f'cannot read the pose of {name}:\n{out}\n{err}'
    return tuple(float(v) for triple in triples[:2] for v in triple)


def sim_seconds(env):
    """Return the current sim time in seconds from the world stats topic."""
    code, out, err = gz(env, 'topic', '-e', '-t', f'/world/{WORLD_NAME}/stats',
                        '-n', '1', timeout=15)
    block = re.search(r'sim_time\s*{([^}]*)}', out)
    assert code == 0 and block, f'cannot read world stats:\n{out}\n{err}'
    sec = re.search(r'\bsec:\s*(\d+)', block.group(1))
    nsec = re.search(r'nsec:\s*(\d+)', block.group(1))
    return (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) / 1e9


def wait_sim_seconds(env, seconds, timeout=120):
    """Block until the sim clock advances `seconds`, whatever the RTF."""
    start = sim_seconds(env)
    poll_until(lambda: sim_seconds(env) - start >= seconds, timeout,
               f'sim advanced less than {seconds}s in {timeout}s of wall time',
               interval=0.5)


def command_boat(env, value, repeats=3):
    """Latch the same normalized command on both BlueBoat propellers over ROS."""
    for _ in range(repeats):
        procs = [subprocess.Popen(
            ['ros2', 'topic', 'pub', '--once', f'/blueboat/motor_{side}/cmd',
             'std_msgs/msg/Float64', f'{{data: {value}}}'],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
            for side in ('port', 'stbd')]
        for proc in procs:
            _, err = proc.communicate(timeout=30)
            assert proc.returncode == 0, f'command failed ({proc.returncode}): {err}'


@pytest.fixture(scope='module')
def sim(request):
    """Bring the demo up headless on isolated domains; return the env."""
    env = dict(os.environ,
               GZ_PARTITION=f'test_{uuid.uuid4().hex[:8]}',
               ROS_DOMAIN_ID=str(int(uuid.uuid4().hex[:2], 16) % 100 + 1),
               ROS_HOME=tempfile.mkdtemp(prefix='kai_multi_vehicle_'))
    launch_sim(request, 'demo launch',
               ['ros2', 'launch', 'kai_bringup', 'multi_vehicle_demo.launch.py',
                'site:=open_water', 'gazebo_gui:=false'],
               env, ready=lambda e: all(f'/{v}/robot_state_publisher' in nodes(e)
                                        for v in VEHICLES), stop=stop)
    poll_until(lambda: all(v in gz(env, 'model', '--list')[1] for v in VEHICLES), 60,
               lambda: f'vehicles missing; models:\n{gz(env, "model", "--list")[1]}')
    return env


def test_each_vehicle_sits_where_it_belongs(sim):
    """Boat at the waterline, ROV under it, quad on the landing pad, all level."""
    wait_sim_seconds(sim, 10)
    for name, (z_expected, tolerance, _) in VEHICLES.items():
        _, _, z, roll, pitch, _ = model_pose(sim, name)
        assert abs(z - z_expected) < tolerance, \
            f'{name} at z {z:+.2f} m, expected {z_expected:+.2f}'
        assert abs(math.degrees(roll)) < 5, f'{name} rolled {math.degrees(roll):+.1f} deg'
        assert abs(math.degrees(pitch)) < 5, f'{name} pitched {math.degrees(pitch):+.1f} deg'


def test_each_vehicle_has_its_own_nodes_and_topics_and_one_clock(sim):
    """A bridge and a state publisher per vehicle in its namespace, topics under it, one clock."""
    needed_nodes = [f'/{name}/{node}' for name in VEHICLES
                    for node in ('ros_gz_bridge', 'robot_state_publisher')]
    poll_until(lambda: all(n in nodes(sim) for n in needed_nodes), 30,
               lambda: f'missing nodes; last listing:\n{nodes(sim)}')
    needed_topics = [f'/{name}/{topic}' for name, (_, _, topics) in VEHICLES.items()
                     for topic in topics]
    poll_until(
        lambda: all(t in ros(sim, 'topic', 'list')[1].split() for t in needed_topics), 30,
        lambda: f'missing topics; last listing:\n{ros(sim, "topic", "list")[1]}')
    listed = ros(sim, 'topic', 'list')[1].split()
    assert '/joint_states' not in listed, 'a vehicle bridges the bare /joint_states'
    code, out, err = ros(sim, 'topic', 'info', '/clock')
    publishers = re.search(r'Publisher count:\s*(\d+)', out)
    assert code == 0 and publishers, f'cannot read /clock info\n{out}\n{err}'
    assert int(publishers.group(1)) == 1, f'/clock published {publishers.group(1)} times'


def test_tf_carries_each_vehicle_under_its_own_prefix(sim):
    """Static TF holds <name>/base_link for every vehicle."""
    out = subprocess.run(
        ['timeout', '15', 'ros2', 'topic', 'echo', '/tf_static',
         '--qos-durability', 'transient_local', '--qos-reliability', 'reliable'],
        env=sim, capture_output=True, text=True, timeout=40).stdout
    for name in VEHICLES:
        assert f'frame_id: {name}/base_link' in out, f'no static TF from {name}/base_link'


def test_a_command_moves_the_boat_and_nothing_else(sim):
    """Half ahead on the boat drives the boat forward; the ROV and the quad stay put."""
    others_before = {name: model_pose(sim, name) for name in ('bluerov2', 'x500')}
    x1, y1, *_ = model_pose(sim, 'blueboat')
    command_boat(sim, 0.5)
    wait_sim_seconds(sim, 8)
    x2, y2, *_ = model_pose(sim, 'blueboat')
    command_boat(sim, 0.0, repeats=2)
    assert math.hypot(x2 - x1, y2 - y1) > 0.5, 'the boat did not move on its command'
    for name, before in others_before.items():
        after = model_pose(sim, name)
        drift = math.hypot(after[0] - before[0], after[1] - before[1])
        assert drift < 0.2, f"{name} moved {drift:.2f} m on the boat's command"
