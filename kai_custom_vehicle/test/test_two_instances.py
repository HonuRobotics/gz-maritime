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
Two custom USVs on the ocean, through the launches a user runs.

sim.launch.xml brings the simulation up with boat_a on it; kai_bringup's
spawn_vehicle.launch.xml then adds boat_b from another process, the way a
second copy of any vehicle joins a running simulation. Both float level at
their waterlines (nothing in the world names either: they float by the
collisions their models mark), each has its own nodes, topics and TF prefix,
the clock has one publisher, and a command on boat_a moves boat_a along its
nose and leaves boat_b where it was. Behaviour is measured in sim time, so
a slow runner changes how long the tests wait, never what they assert.
"""

import functools
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import uuid

from ament_index_python.packages import get_package_share_directory
from conftest import launch_sim, make_cli, poll_until, stop_process_group
import pytest

SHARE = Path(get_package_share_directory('kai_custom_vehicle'))
WORLD_NAME = 'default'
BOATS = ('boat_a', 'boat_b')
SPAWN = ['ros2', 'launch', 'kai_bringup', 'spawn_vehicle.launch.xml',
         f'xacro:={SHARE / "models" / "custom_usv" / "model.sdf.xacro"}',
         f'bridge:={SHARE / "config" / "ros_gz_bridge.yaml.in"}',
         f'urdf:={SHARE / "urdf" / "custom_usv.urdf.xacro"}']

_NUM = r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?'
POSE_TRIPLE = re.compile(rf'({_NUM}) ({_NUM}) ({_NUM})')

gz = make_cli('gz')
ros = make_cli('ros2', default_timeout=20)
# SIGINT first so ros2 launch shuts its children down in order.
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
    # \b so 'sec:' cannot match inside 'nsec:' when sec is omitted (== 0).
    sec = re.search(r'\bsec:\s*(\d+)', block.group(1))
    nsec = re.search(r'nsec:\s*(\d+)', block.group(1))
    return (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) / 1e9


def wait_sim_seconds(env, seconds, timeout=120):
    """Block until the sim clock advances `seconds`, whatever the RTF."""
    start = sim_seconds(env)
    poll_until(lambda: sim_seconds(env) - start >= seconds, timeout,
               f'sim advanced less than {seconds}s in {timeout}s of wall time',
               interval=0.5)


def command_motors(env, name, mapping, repeats=3):
    """
    Latch normalized commands on one boat over ROS, both topics at once.

    Parallel publication matters: commands latch, so staggered onset applies a
    differential wrench and yaws the boat off its heading. Rounds repeat
    because a one-shot publication can lose the discovery race.
    """
    for _ in range(repeats):
        procs = [(side, subprocess.Popen(
            ['ros2', 'topic', 'pub', '--once', f'/{name}/motor_{side}/cmd',
             'std_msgs/msg/Float64', f'{{data: {value}}}'],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True))
            for side, value in mapping.items()]
        for side, proc in procs:
            _, err = proc.communicate(timeout=30)
            assert proc.returncode == 0, f'motor {side} command failed ({proc.returncode}): {err}'


@pytest.fixture(scope='module')
def sim(request):
    """Bring up sim.launch.xml with boat_a, spawn boat_b next to it; return the env."""
    env = dict(os.environ,
               GZ_PARTITION=f'test_{uuid.uuid4().hex[:8]}',
               # uuid, not pid: parallel colcon test runs could share a domain
               # when two pids agree modulo 100.
               ROS_DOMAIN_ID=str(int(uuid.uuid4().hex[:2], 16) % 100 + 1),
               # The instance files land under $ROS_HOME/kai_bringup/<name>.
               ROS_HOME=tempfile.mkdtemp(prefix='kai_custom_vehicle_'))
    launch_sim(request, 'sim launch',
               ['ros2', 'launch', 'kai_custom_vehicle', 'sim.launch.xml',
                'gazebo_gui:=false', 'name:=boat_a', 'y:=2'],
               env, ready=lambda e: '/boat_a/robot_state_publisher' in nodes(e), stop=stop)
    # With composition the spawn launch loads its bridge and state publisher
    # into the simulation's container and exits once the model is created;
    # its nodes live on in there, and go down with the simulation launch.
    spawn = subprocess.run(SPAWN + ['name:=boat_b', 'y:=-2', 'yaw:=1.57'], env=env,
                           capture_output=True, text=True, timeout=120)
    assert spawn.returncode == 0, f'spawn launch failed:\n{spawn.stdout}\n{spawn.stderr}'
    poll_until(lambda: '/boat_b/robot_state_publisher' in nodes(env), 60,
               lambda: f'boat_b never came up; nodes:\n{nodes(env)}\n{spawn.stdout}')
    poll_until(lambda: all(n in gz(env, 'model', '--list')[1] for n in BOATS), 30,
               'the boats never appeared in the world')
    return env


def test_both_boats_float_level_at_their_waterlines(sim):
    """
    Each boat settles with base_link at z = 0, level, where it was spawned.

    Nothing in the world names either boat: they float by the collisions
    their models mark. An unmarked boat would be tens of metres down by now.
    """
    wait_sim_seconds(sim, 10)
    for name, y in zip(BOATS, (2.0, -2.0)):
        _, ym, z, roll, pitch, _ = model_pose(sim, name)
        assert abs(z) < 0.02, f'{name}: base_link at z {z:+.3f} m, not at the waterline'
        assert abs(ym - y) < 0.1, f'{name}: at y {ym:+.2f} m, spawned at {y:+.1f}'
        assert abs(math.degrees(roll)) < 2, f'{name} rolled {math.degrees(roll):+.1f} deg'
        assert abs(math.degrees(pitch)) < 2, f'{name} pitched {math.degrees(pitch):+.1f} deg'


def test_each_boat_has_its_own_nodes_and_topics_and_one_clock(sim):
    """
    Bridge and state publisher per instance, topics per instance, /clock once.

    The clock comes from the simulation launch; a vehicle's bridge must not
    publish it again.
    """
    needed_nodes = [f'/{name}/{node}' for name in BOATS
                    for node in ('ros_gz_bridge', 'robot_state_publisher')]
    poll_until(lambda: all(n in nodes(sim) for n in needed_nodes), 30,
               lambda: f'missing nodes; last listing:\n{nodes(sim)}')
    needed_topics = [f'/{name}/{topic}' for name in BOATS
                     for topic in ('motor_port/cmd', 'motor_stbd/cmd',
                                   'motor_port/cmd/ang_vel', 'joint_states',
                                   'imu', 'magnetometer', 'navsat', 'robot_description')]
    poll_until(
        lambda: all(t in ros(sim, 'topic', 'list')[1].split() for t in needed_topics), 30,
        lambda: f'missing topics; last listing:\n{ros(sim, "topic", "list")[1]}')
    code, out, err = ros(sim, 'topic', 'info', '/clock')
    publishers = re.search(r'Publisher count:\s*(\d+)', out)
    assert code == 0 and publishers, f'cannot read /clock info\n{out}\n{err}'
    assert int(publishers.group(1)) == 1, f'/clock published {publishers.group(1)} times'


def test_tf_carries_each_boat_under_its_own_prefix(sim):
    """
    Static TF holds boat_a/base_link and boat_b/base_link, each with its frames.

    frame_prefix in each robot_state_publisher is what keeps two identical
    URDFs from publishing one tangled tree.
    """
    out = subprocess.run(
        ['timeout', '15', 'ros2', 'topic', 'echo', '/tf_static',
         '--qos-durability', 'transient_local', '--qos-reliability', 'reliable'],
        env=sim, capture_output=True, text=True, timeout=40).stdout
    for name in BOATS:
        assert f'frame_id: {name}/base_link' in out, f'no static TF from {name}/base_link'
        assert f'child_frame_id: {name}/imu_link' in out, f'no {name}/imu_link in TF'


def test_command_moves_one_boat_along_its_nose_and_not_the_other(sim):
    """
    Half ahead on boat_a drives boat_a forward; boat_b stays put.

    The assertion on boat_a is on the steady state, which is what the model
    owns: velocity aligned with the body x axis and no residual yaw. The one
    on boat_b is what separate topics own: it does not move.
    """
    other_before = model_pose(sim, 'boat_b')
    command_motors(sim, 'boat_a', {'port': 0.5, 'stbd': 0.5})
    wait_sim_seconds(sim, 5)          # onset transient: speed builds
    t1 = sim_seconds(sim)
    x1, y1, _, _, _, yaw1 = model_pose(sim, 'boat_a')
    wait_sim_seconds(sim, 5)
    t2 = sim_seconds(sim)
    x2, y2, _, _, _, yaw2 = model_pose(sim, 'boat_a')
    command_motors(sim, 'boat_a', {'port': 0.0, 'stbd': 0.0}, repeats=2)
    dx, dy = x2 - x1, y2 - y1
    speed = math.hypot(dx, dy) / (t2 - t1)
    assert speed > 0.1, f'no surge: {speed:.3f} m/s over {t2 - t1:.1f} sim s'
    crab = (math.degrees(math.atan2(dy, dx) - yaw2) + 180) % 360 - 180
    assert abs(crab) < 15, (f'not moving along the nose: crab {crab:+.1f} deg, '
                            f'travel ({dx:+.2f},{dy:+.2f}) m at yaw {math.degrees(yaw2):+.1f} deg')
    residual_yaw = math.degrees(yaw2 - yaw1)
    assert abs(residual_yaw) < 15, f'still yawing in steady state: {residual_yaw:+.1f} deg'
    other_after = model_pose(sim, 'boat_b')
    drift = math.hypot(other_after[0] - other_before[0], other_after[1] - other_before[1])
    assert drift < 0.1, f"boat_b moved {drift:.2f} m on boat_a's commands"
