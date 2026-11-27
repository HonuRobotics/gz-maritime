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
The acoustic pinger demo, through the launch a user runs, in two worlds.

acoustic_demo.launch.xml brings up a world, headless, with custom_usv homing
on the pinger it hears and boat_b listening elsewhere. In each world a green
buoy's 30 kHz pinger pings and a red one's is silent:

* kai_gazebo's acoustic_demo.sdf: custom_usv at the origin heading east,
  boat_b at (-10, -20) heading north, green at (40, 15), red at (40, -15),
  and a yellow 37 kHz decoy at (-30, 30);
* marina_bay_lite.sdf, the RobotX 2026 course: custom_usv at course B's
  start point, boat_b at course C's, the survey pingers 35 m out from B's.
  Its site comes from Fuel, so this one runs only once the site is cached.

The tests check what the boats hear, that custom_usv reaches the green
buoy, and that it turns for the red one once the pingers are switched.
Progress is measured in sim time, so a slow runner changes how long the
tests wait, never what they assert.
"""

import functools
import math
import os
from pathlib import Path
import re
import signal
import statistics
import tempfile
import uuid

from conftest import launch_sim, make_cli, poll_until, stop_process_group
import pytest

WORLD_NAME = 'default'

# Each world: its launch arguments, the green and red buoys, boat_b's pose,
# and the pingers its ground truth lists.
SCENES = {
    'acoustic_demo': {
        'args': [],
        'green': (40.0, 15.0),
        'red': (40.0, -15.0),
        'boat_b': (-10.0, -20.0, math.pi / 2),
        'pingers': ('pinger_green', 'pinger_red', 'pinger_yellow'),
    },
    'marina_bay_lite': {
        'args': ['world:=marina_bay_lite.sdf', 'x:=-15', 'y:=-8', 'yaw:=0.55',
                 'listener_x:=15', 'listener_y:=-28', 'listener_yaw:=0.55'],
        'green': (12.2, 14.6),
        'red': (17.5, 6.0),
        'boat_b': (15.0, -28.0, 0.55),
        'pingers': ('pinger_green', 'pinger_red'),
        # The site, from Fuel: only once it is in the cache.
        'needs': Path(os.environ.get('GZ_FUEL_CACHE_PATH',
                                     Path.home() / '.gz' / 'fuel')) /
        'fuel.gazebosim.org' / 'honurobotics' / 'models' / 'marina_bay_lite',
    },
}
# How close to a buoy counts as there: the controller stops when the median
# of the last pings is louder than -16 dB, about 6 m away, give or take the
# receiver's 1 dB of amplitude noise.
ARRIVED = 8.0

_NUM = r'-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?'
POSE_TRIPLE = re.compile(rf'({_NUM}) ({_NUM}) ({_NUM})')

gz = make_cli('gz')
ros = make_cli('ros2', default_timeout=20)
stop = functools.partial(stop_process_group, sig=signal.SIGINT, grace=20)


def nodes(env):
    """Return the ROS node names."""
    return ros(env, 'node', 'list')[1].split()


def model_xy(env, name):
    """
    Return a model's world position (x, y) from `gz model -p`, or None.

    None when the call fails, as it does when the machine is busy and the
    service times out: the caller polls, and tries again.
    """
    code, out, _ = gz(env, 'model', '-m', name, '-p', timeout=15)
    triples = POSE_TRIPLE.findall(out)
    if code != 0 or not triples:
        return None
    return float(triples[0][0]), float(triples[0][1])


def distance_to(env, name, point):
    """Return the horizontal distance from a model to a point, m, or None."""
    xy = model_xy(env, name)
    if xy is None:
        return None
    return math.hypot(xy[0] - point[0], xy[1] - point[1])


def sim_seconds(env):
    """Return the sim time in seconds from the world stats topic, or None."""
    code, out, _ = gz(env, 'topic', '-e', '-t', f'/world/{WORLD_NAME}/stats',
                      '-n', '1', timeout=15)
    block = re.search(r'sim_time\s*{([^}]*)}', out)
    if code != 0 or not block:
        return None
    sec = re.search(r'\bsec:\s*(\d+)', block.group(1))
    nsec = re.search(r'nsec:\s*(\d+)', block.group(1))
    return (int(sec.group(1)) if sec else 0) + (int(nsec.group(1)) if nsec else 0) / 1e9


def wait_until_near(env, name, point, sim_budget, wall_timeout=300):
    """
    Block until a model is within ARRIVED of a point, within sim_budget s of sim time.

    A call that fails is not a failure of the test: every reading is polled
    for, so a slow answer on a busy machine is asked for again.
    """
    seen = {'failing': 0}

    def started():
        seen['start'] = sim_seconds(env)
        return seen['start'] is not None

    def near_or_out_of_time():
        seen['d'] = distance_to(env, name, point)
        if seen['d'] is not None and seen['d'] < ARRIVED:
            return True
        now = sim_seconds(env)
        if now is not None:
            seen['now'] = now
        # Polls in a row where a gz call failed, to tell a broken gz from a
        # stalled simulation when the time runs out.
        seen['failing'] = seen['failing'] + 1 if seen['d'] is None or now is None else 0
        return seen['d'] is not None and now is not None and now - seen['start'] > sim_budget

    def why():
        if seen['failing'] > 0:
            return (f"gz calls failed for the last {seen['failing']} polls while "
                    f'waiting for {name}: the pose or the world stats cannot be read')
        return (f'sim time stalled while waiting for {name}, at '
                f"{seen.get('now', seen['start']):.1f} s")

    poll_until(started, 60, 'cannot read world stats', interval=1)
    poll_until(near_or_out_of_time, wall_timeout, why, interval=1)
    assert seen['d'] < ARRIVED, (
        f"{name} is {seen['d']:.1f} m from {point} after "
        f"{seen.get('now', seen['start']) - seen['start']:.0f} s of sim time")


def gz_readings(env, topic, count):
    """Return `count` receiver messages from the Gazebo side, as dicts of their numbers."""
    code, out, err = gz(env, 'topic', '-e', '-t', topic, '-n', str(count), timeout=30)
    assert code == 0, f'no readings on {topic}:\n{out}\n{err}'
    readings = []
    for msg in re.split(r'\n(?=header \{)', out.strip()):
        values = {k: float(v) for k, v in re.findall(
            rf'key: "(\w+)"\s*value \{{[^}}]*?double_value: ({_NUM})', msg)}
        values['valid'] = bool(re.search(r'key: "valid"\s*value \{[^}]*bool_value: true', msg))
        readings.append(values)
    assert len(readings) == count, f'expected {count} readings on {topic}:\n{out}'
    return readings


def set_pinger(env, name, active):
    """Switch a pinger on or off over ROS, through the demo's bridge."""
    value = 'true' if active else 'false'
    code, out, err = ros(
        env, 'topic', 'pub', '--once', '-w', '1',
        f'/world/{WORLD_NAME}/acoustics/{name}/set', 'ros_gz_interfaces/msg/ParamVec',
        f'{{params: [{{name: active, value: {{type: 1, bool_value: {value}}}}}]}}',
        timeout=30)
    assert code == 0, f'cannot switch {name}:\n{out}\n{err}'


@pytest.fixture(scope='module', params=sorted(SCENES))
def demo(request):
    """Bring up the acoustic demo in a world, headless; return the env and the scene."""
    scene = SCENES[request.param]
    if 'needs' in scene and not scene['needs'].is_dir():
        pytest.skip(f'{request.param} needs its Fuel site cached at {scene["needs"]}')
    home = tempfile.mkdtemp(prefix='kai_acoustic_demo_')
    env = dict(os.environ,
               GZ_PARTITION=f'test_{uuid.uuid4().hex[:8]}',
               ROS_DOMAIN_ID=str(int(uuid.uuid4().hex[:2], 16) % 100 + 1),
               ROS_HOME=home)
    launch_sim(request, 'acoustic demo',
               ['ros2', 'launch', 'kai_custom_vehicle', 'acoustic_demo.launch.xml',
                'gazebo_gui:=false', *scene['args']],
               env,
               ready=lambda e: {'/custom_usv/pinger_homing',
                                '/boat_b/robot_state_publisher'} <= set(nodes(e)),
               stop=stop)
    return env, scene


def test_the_listener_hears_the_green_pinger_from_where_it_is(demo):
    """
    boat_b hears the green pinger along its bearing from boat_b, not a decoy.

    In the demo world, from (-10, -20) heading north, the green buoy at
    (40, 15) is 55 degrees to starboard. The median of a few pings throws
    out an outlier.
    """
    env, scene = demo
    readings = [r for r in gz_readings(env, '/boat_b/acoustic_receiver', 5) if r['valid']]
    assert len(readings) >= 3, readings
    assert all(r['frequency'] == 30000.0 for r in readings), readings
    green, boat_b = scene['green'], scene['boat_b']
    dx, dy = green[0] - boat_b[0], green[1] - boat_b[1]
    expected = math.atan2(dy, dx) - boat_b[2]
    bearing = statistics.median(r['bearing'] for r in readings)
    assert bearing == pytest.approx(expected, abs=0.15), readings
    # Spherical spreading from where it is, under the receiver's 1 dB of
    # amplitude noise.
    amplitude = statistics.median(r['amplitude'] for r in readings)
    assert amplitude == pytest.approx(-20 * math.log10(math.hypot(dx, dy)), abs=2.5)


def test_the_readings_reach_ros(demo):
    """The vehicle bridge carries a reading as a ParamVec, in the hydrophone frame."""
    env, _ = demo
    code, out, err = ros(env, 'topic', 'echo', '--once', '/boat_b/acoustic_receiver',
                         timeout=30)
    assert code == 0, f'no reading over ROS:\n{out}\n{err}'
    assert 'frame_id: boat_b/hydrophone_link' in out, out
    for key in ('bearing', 'elevation', 'frequency', 'amplitude', 'valid'):
        assert f'name: {key}' in out, f'{key} missing:\n{out}'
    # A passive receiver: neither the range nor the time the ping was sent,
    # which with the stamp gives the range.
    for key in ('range', 'emit_time'):
        assert f'name: {key}' not in out, f'{key} reported:\n{out}'


def test_the_ground_truth_lists_the_pingers(demo):
    """Every pinger, by name, on the world's ground truth topic, bridged to ROS."""
    env, scene = demo
    code, out, err = ros(env, 'topic', 'echo', '--once',
                         f'/world/{WORLD_NAME}/acoustics/pingers', timeout=30)
    assert code == 0, f'no ground truth over ROS:\n{out}\n{err}'
    for name in scene['pingers']:
        assert f'string_value: {name}' in out, f'{name} missing:\n{out}'


def test_the_homing_boat_reaches_the_green_buoy(demo):
    """custom_usv homes on the only 30 kHz pinger pinging, 25 to 43 m away."""
    env, scene = demo
    wait_until_near(env, 'custom_usv', scene['green'], sim_budget=90)


def test_switching_pingers_sends_it_to_the_red_buoy(demo):
    """With green off and red on, custom_usv turns and crosses to red."""
    env, scene = demo
    set_pinger(env, 'pinger_green', False)
    set_pinger(env, 'pinger_red', True)
    wait_until_near(env, 'custom_usv', scene['red'], sim_budget=120)
