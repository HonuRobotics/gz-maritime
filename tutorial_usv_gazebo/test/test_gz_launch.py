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
Headless Gazebo integration: two boats on the open_water world.

Runs gz directly, no ROS: the world comes up with its wave service, two
instances generated under different names are spawned into it the way any
vehicle is, and both float level at their waterlines. Floating proves the
marks: the world floats nothing it is not told about, and nothing in it
names either boat. Thrust on one boat then moves that boat along its nose
and leaves the other where it was, which proves the topics are separate.
Behavior is measured in sim time, so a slow runner changes how long the
tests wait, never what they assert.
"""

import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
import uuid

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from conftest import launch_sim, make_cli, poll_until
import pytest

WORLD = Path(get_package_share_directory('kai_gazebo')) / 'worlds' / 'open_water.sdf'
WORLD_NAME = 'default'
SCRIPT = (Path(get_package_prefix('tutorial_usv_gazebo'))
          / 'lib' / 'tutorial_usv_gazebo' / 'configure_vehicle.py')
BOATS = ('boat_a', 'boat_b')

_NUM = r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?'
POSE_TRIPLE = re.compile(rf'({_NUM}) ({_NUM}) ({_NUM})')

gz = make_cli('gz')


def services(env):
    """Return the advertised gz service names."""
    return gz(env, 'service', '-l')[1].split()


def model_pose(env, name):
    """Return a model's world pose from `gz model -p`: (x, y, z, roll, pitch, yaw)."""
    code, out, err = gz(env, 'model', '-m', name, '-p', timeout=15)
    triples = POSE_TRIPLE.findall(out)
    assert code == 0 and len(triples) >= 2, (
        f'cannot read the pose of {name}:\n{out}\n{err}')
    return tuple(float(v) for triple in triples[:2] for v in triple)


def sim_seconds(env):
    """Return the current sim time in seconds from the world stats topic."""
    code, out, err = gz(env, 'topic', '-e', '-t',
                        f'/world/{WORLD_NAME}/stats', '-n', '1', timeout=15)
    block = re.search(r'sim_time\s*{([^}]*)}', out)
    assert code == 0 and block, f'cannot read world stats:\n{out}\n{err}'
    # \b so 'sec:' cannot match inside 'nsec:' when sec is omitted (== 0).
    sec = re.search(r'\bsec:\s*(\d+)', block.group(1))
    nsec = re.search(r'nsec:\s*(\d+)', block.group(1))
    return (int(sec.group(1)) if sec else 0) + \
        (int(nsec.group(1)) if nsec else 0) / 1e9


def wait_sim_seconds(env, seconds, timeout=120):
    """Block until the sim clock advances `seconds`, whatever the RTF."""
    start = sim_seconds(env)
    poll_until(lambda: sim_seconds(env) - start >= seconds, timeout,
               f'sim advanced less than {seconds}s in {timeout}s of wall time',
               interval=0.5)


def command_motors(env, name, mapping, repeats=6):
    """
    Latch thrust commands (N) on one boat, every topic in parallel each round.

    Parallel publication matters: commands latch, so staggered onset applies a
    differential wrench and yaws the boat off its heading. Rounds repeat
    because one-shot publications can lose the discovery race.
    """
    for _ in range(repeats):
        procs = [(side, subprocess.Popen(
            ['gz', 'topic', '-t', f'/{name}/motor_{side}/thrust',
             '-m', 'gz.msgs.Double', '-p', f'data: {value}'],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            text=True)) for side, value in mapping.items()]
        for side, proc in procs:
            _, err = proc.communicate(timeout=20)
            assert proc.returncode == 0, (
                f'motor {side} command failed ({proc.returncode}): {err}')


@pytest.fixture(scope='module')
def sim(request):
    """Start a headless open_water server on an isolated partition; yield its env."""
    env = dict(os.environ, GZ_PARTITION=f'test_{uuid.uuid4().hex[:8]}')
    # -v 3 so warnings and messages (not just errors) reach the logged output.
    return launch_sim(
        request, 'gz sim',
        ['gz', 'sim', '-s', '-r', '-v', '3', str(WORLD)], env,
        ready=lambda e: f'/world/{WORLD_NAME}/wave/set_parameters' in services(e))


@pytest.fixture(scope='module')
def boats(sim):
    """Generate and spawn two instances, 10 cm above the water, 4 m apart."""
    for index, name in enumerate(BOATS):
        out_dir = Path(tempfile.mkdtemp(prefix=f'{name}_'))
        result = subprocess.run(
            [str(SCRIPT), '--name', name, '--out-dir', str(out_dir)],
            capture_output=True, text=True, timeout=120)
        assert result.returncode == 0, result.stderr
        req = (f'sdf_filename: "{out_dir / "model.sdf"}", name: "{name}", '
               f'pose: {{position: {{y: {2 - 4 * index}, z: 0.1}}}}')
        code, out, err = gz(sim, 'service', '-s', f'/world/{WORLD_NAME}/create',
                            '--reqtype', 'gz.msgs.EntityFactory',
                            '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '10000', '--req', req, timeout=30)
        assert code == 0 and 'true' in out, f'spawning {name} failed:\n{out}\n{err}'
    poll_until(lambda: all(n in gz(sim, 'model', '--list')[1] for n in BOATS), 30,
               'the boats never appeared in the world')
    return sim


def test_wave_service_advertised(sim):
    """The world runs the wave source, and no buoyancy registration service."""
    listed = services(sim)
    assert f'/world/{WORLD_NAME}/wave/set_parameters' in listed
    assert not [s for s in listed if 'buoyancy' in s], listed


def test_both_boats_float_level_at_their_waterlines(boats):
    """
    Dropped from 10 cm, each boat settles with base_link at z = 0, level.

    Nothing in the world names either boat: they float by the collisions
    their models mark. An unmarked boat would be tens of metres down by now.
    """
    wait_sim_seconds(boats, 10)
    for name in BOATS:
        _, _, z, roll, pitch, _ = model_pose(boats, name)
        assert abs(z) < 0.02, f'{name}: base_link at z {z:+.3f} m, not at the waterline'
        assert abs(math.degrees(roll)) < 2, f'{name} rolled {math.degrees(roll):+.1f} deg'
        assert abs(math.degrees(pitch)) < 2, f'{name} pitched {math.degrees(pitch):+.1f} deg'


def test_each_boat_has_its_own_interfaces(boats):
    """Thrust commands, propeller speeds, joint states and sensors, per boat."""
    needed = []
    for name in BOATS:
        needed += [f'/{name}/motor_{side}/thrust{suffix}'
                   for side in ('port', 'stbd') for suffix in ('', '/ang_vel')]
        needed += [f'/{name}/{topic}'
                   for topic in ('joint_states', 'imu', 'magnetometer', 'navsat')]
    poll_until(
        lambda: all(t in gz(boats, 'topic', '-l')[1].split() for t in needed), 30,
        lambda: f'missing topics; last listing:\n{gz(boats, "topic", "-l")[1]}')


def test_thrust_moves_one_boat_along_its_nose_and_not_the_other(boats):
    """
    Equal thrust on boat_a drives boat_a forward; boat_b stays put.

    The assertion on boat_a is on the steady state, which is what the model
    owns: velocity aligned with the body x axis and no residual yaw. The one
    on boat_b is what separate topics own: it does not move.
    """
    other_before = model_pose(boats, 'boat_b')
    command_motors(boats, 'boat_a', {'port': 5.0, 'stbd': 5.0})
    wait_sim_seconds(boats, 5)          # onset transient: speed builds
    t1 = sim_seconds(boats)
    x1, y1, _, _, _, yaw1 = model_pose(boats, 'boat_a')
    wait_sim_seconds(boats, 5)
    t2 = sim_seconds(boats)
    x2, y2, _, _, _, yaw2 = model_pose(boats, 'boat_a')
    command_motors(boats, 'boat_a', {'port': 0.0, 'stbd': 0.0}, repeats=2)
    dx, dy = x2 - x1, y2 - y1
    speed = math.hypot(dx, dy) / (t2 - t1)
    assert speed > 0.1, f'no surge: {speed:.3f} m/s over {t2 - t1:.1f} sim s'
    crab = math.degrees(math.atan2(dy, dx) - yaw2)
    crab = (crab + 180) % 360 - 180
    assert abs(crab) < 15, (
        f'not moving along the nose: crab {crab:+.1f} deg, '
        f'travel ({dx:+.2f},{dy:+.2f}) m at yaw {math.degrees(yaw2):+.1f} deg')
    residual_yaw = math.degrees(yaw2 - yaw1)
    assert abs(residual_yaw) < 15, (
        f'still yawing in steady state: {residual_yaw:+.1f} deg over the window')
    other_after = model_pose(boats, 'boat_b')
    drift = math.hypot(other_after[0] - other_before[0],
                       other_after[1] - other_before[1])
    assert drift < 0.1, f"boat_b moved {drift:.2f} m on boat_a's commands"
