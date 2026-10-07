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
Custom USVs in an ocean current, through the launches a user runs.

open_water.sdf is copied with its ocean current set to 0.5 m/s setting east
(towards +x). sim.launch.xml brings it up with drifter, engines off, pointing
east; holder is spawned pointing west, into the current, with the command
that balances it; latecomer is spawned once the others have been in the water
a while. The water pushes on each boat's pontoons, marked
gz:ocean_current="true", relative to the water. The drifter and the latecomer
settle at the current's velocity, and the holder stays put. Behaviour is
measured in sim time, so a slow runner changes how long the tests wait, never
what they assert.
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
WORLDS = Path(get_package_share_directory('kai_gazebo')) / 'worlds'
WORLD_NAME = 'default'
SPAWN = ['ros2', 'launch', 'kai_bringup', 'spawn_vehicle.launch.xml',
         f'xacro:={SHARE / "models" / "custom_usv" / "model.sdf.xacro"}',
         f'bridge:={SHARE / "config" / "ros_gz_bridge.yaml.in"}',
         f'urdf:={SHARE / "urdf" / "custom_usv.urdf.xacro"}']

# The current: 0.5 m/s setting east, the world's +x.
CURRENT_SPEED = 0.5
CURRENT = (CURRENT_SPEED, 0.0)

# The custom USV's pontoons, from kai_custom_vehicle/urdf/dimensions.xacro:
# two boxes 1.0 m long and 0.15 m wide, floating a 12.1 kg boat in seawater
# at a draft of 12.1 / (1025 * 2 * 1.0 * 0.15), about 3.9 cm.
WATER_DENSITY = 1025.0
DRAFT = 12.1 / (WATER_DENSITY * 2 * 1.0 * 0.15)

# What the current sees head on: the wetted part of both pontoons' 0.15 m
# wide ends, with the marks' drag coefficient of 1.
WETTED_FRONT = 2 * 0.15 * DRAFT

# The command that holds the custom USV still, pointing into the current.
# Through the water it makes the current's speed, where the drag on the
# marked pontoons is 0.5 * rho * Cd * A * u**2, about 1.5 N, shared by two
# motors, and a command of 1 is 20 N.
HOLD = 0.5 * WATER_DENSITY * WETTED_FRONT * CURRENT_SPEED ** 2 / 2 / 20.0

# Quadratic drag alone has no linear term, so a boat starting at rest
# catches up with the current slowly: its shortfall is 1 / (1 / u + k t),
# k = 0.5 * rho * Cd * A / m, about 0.5 1/m here, under 0.05 m/s after
# about 36 s. A minute leaves margin.
SETTLE = 60

# How close to the expected ground velocity a settled boat must be, m/s.
TOL = 0.05

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


def ground_velocity(env, name, seconds=5):
    """Return a boat's mean horizontal velocity over the ground, (vx, vy) m/s."""
    t1 = sim_seconds(env)
    x1, y1 = model_pose(env, name)[:2]
    wait_sim_seconds(env, seconds)
    t2 = sim_seconds(env)
    x2, y2 = model_pose(env, name)[:2]
    return (x2 - x1) / (t2 - t1), (y2 - y1) / (t2 - t1)


def command_motors(env, name, value, repeats=3):
    """
    Latch one normalized command on both motors of a boat, over ROS.

    Parallel publication matters: commands latch, so staggered onset applies a
    differential wrench and yaws the boat off its heading. Rounds repeat
    because a one-shot publication can lose the discovery race.
    """
    for _ in range(repeats):
        procs = [(side, subprocess.Popen(
            ['ros2', 'topic', 'pub', '--once', f'/{name}/motor_{side}/cmd',
             'std_msgs/msg/Float64', f'{{data: {value}}}'],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True))
            for side in ('port', 'stbd')]
        for side, proc in procs:
            _, err = proc.communicate(timeout=30)
            assert proc.returncode == 0, f'motor {side} command failed ({proc.returncode}): {err}'


def world_with_current(directory):
    """Write open_water.sdf with its ocean current set; return the new file."""
    sdf = (WORLDS / 'open_water.sdf').read_text()
    start = sdf.index('filename="gz-maritime-ocean-current-system"')
    end = sdf.index('</plugin>', start)
    block = sdf[start:end]
    block = re.sub(r'<speed>[^<]*</speed>', f'<speed>{CURRENT_SPEED}</speed>', block)
    block = re.sub(r'<direction>[^<]*</direction>', '<direction>90</direction>', block)
    path = Path(directory) / 'open_water_current.sdf'
    path.write_text(sdf[:start] + block + sdf[end:])
    return path


def spawn(env, name, *args):
    """Spawn one more custom USV into the running simulation."""
    out = subprocess.run(SPAWN + [f'name:={name}', *args], env=env,
                         capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, f'spawn launch failed:\n{out.stdout}\n{out.stderr}'
    poll_until(lambda: f'/{name}/robot_state_publisher' in nodes(env), 60,
               lambda: f'{name} never came up; nodes:\n{nodes(env)}\n{out.stdout}')
    poll_until(lambda: name in gz(env, 'model', '--list')[1], 30,
               f'{name} never appeared in the world')


@pytest.fixture(scope='module')
def sim(request):
    """Bring up the current world with drifter, spawn holder; return the env."""
    home = tempfile.mkdtemp(prefix='kai_custom_vehicle_')
    env = dict(os.environ,
               GZ_PARTITION=f'test_{uuid.uuid4().hex[:8]}',
               ROS_DOMAIN_ID=str(int(uuid.uuid4().hex[:2], 16) % 100 + 1),
               ROS_HOME=home)
    world = world_with_current(home)
    launch_sim(request, 'sim launch',
               ['ros2', 'launch', 'kai_custom_vehicle', 'sim.launch.xml',
                'gazebo_gui:=false', f'world:={world}', 'name:=drifter', 'y:=4'],
               env, ready=lambda e: '/drifter/robot_state_publisher' in nodes(e), stop=stop)
    spawn(env, 'holder', 'y:=-4', f'yaw:={math.pi}')
    command_motors(env, 'holder', HOLD)
    return env


def test_the_world_has_the_current(sim):
    """The ground truth reads the current the world file was given."""
    code, out, err = gz(sim, 'topic', '-e', '-t',
                        f'/world/{WORLD_NAME}/ocean_current_info', '-n', '1', timeout=15)
    assert code == 0, f'no ocean current ground truth:\n{out}\n{err}'
    x = re.search(rf'linear\s*{{[^}}]*x:\s*({_NUM})', out)
    assert x and float(x.group(1)) == pytest.approx(CURRENT_SPEED), out


def test_a_boat_with_its_engines_off_drifts_with_the_current(sim):
    """
    The drifter settles at the current's velocity over the ground.

    The marks drag its pontoons against the water; a hull damped against the
    ground, as Gazebo's own hydrodynamics does, would not move at all.
    """
    wait_sim_seconds(sim, SETTLE)      # onset: it catches up with the current
    vx, vy = ground_velocity(sim, 'drifter')
    assert vx == pytest.approx(CURRENT[0], abs=TOL), f'drifting at ({vx:+.3f},{vy:+.3f}) m/s'
    assert vy == pytest.approx(CURRENT[1], abs=TOL), f'drifting at ({vx:+.3f},{vy:+.3f}) m/s'


def test_a_boat_making_the_currents_speed_upstream_holds_station(sim):
    """
    The holder, pointing into the current, makes 0.5 m/s through the water.

    The thrust it takes is the drag the current puts on its wetted pontoons
    at that speed, so over the ground it barely moves.
    """
    wait_sim_seconds(sim, 10)
    vx, vy = ground_velocity(sim, 'holder')
    assert math.hypot(vx, vy) < TOL, f'holder moving at ({vx:+.3f},{vy:+.3f}) m/s'


def test_a_boat_spawned_later_drifts_the_same(sim):
    """A third boat, spawned into the running current, drifts like the first."""
    spawn(sim, 'latecomer', 'y:=-12')
    wait_sim_seconds(sim, SETTLE)
    vx, vy = ground_velocity(sim, 'latecomer')
    assert vx == pytest.approx(CURRENT[0], abs=TOL), f'drifting at ({vx:+.3f},{vy:+.3f}) m/s'
    assert vy == pytest.approx(CURRENT[1], abs=TOL), f'drifting at ({vx:+.3f},{vy:+.3f}) m/s'
