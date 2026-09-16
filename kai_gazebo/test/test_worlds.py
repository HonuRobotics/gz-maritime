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
The worlds are well formed, pass the SDF check and run headless.

Every world is parsed and checked for the contract the docs state: the
name every command relies on, the systems every vehicle needs, gz-maritime's
buoyancy naming nobody, a wave source and the drawn sea. The worlds with no
Fuel model in them, the open sea and La Spezia, also pass the SDF checker
with every include resolved, and La Spezia runs for a few hundred steps
without rendering, so a broken mesh path or a plugin that fails to load
shows up here rather than at the first launch. The other three need Fuel
(the Sydney and Benderson Park terrains, the Sand Island shore camp), a
download on first use that has no place in a unit test.
"""

import os
from pathlib import Path
import subprocess
import uuid
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

WORLDS = Path(get_package_share_directory('kai_gazebo')) / 'worlds'
ALL = ('open_water', 'sydney_regatta', 'benderson_park', 'sand_island', 'la_spezia')
# Worlds with no Fuel model in them, so they load with no network and no cache.
# Sand Island's terrain is local, but its shore camp comes from Fuel.
LOCAL = ('open_water', 'la_spezia')
LOCAL_TERRAIN = ('la_spezia',)
REQUIRED_SYSTEMS = (
    'gz-sim-physics-system', 'gz-sim-user-commands-system',
    'gz-sim-scene-broadcaster-system', 'gz-sim-sensors-system',
    'gz-sim-imu-system', 'gz-sim-magnetometer-system', 'gz-sim-navsat-system',
    'gz-sim-air-pressure-system', 'gz-maritime-buoyancy-system',
)


def sdf_env():
    """Return the gz CLI environment: model:// resolves through SDF_PATH, not the sim path."""
    env = dict(os.environ)
    env['SDF_PATH'] = os.pathsep.join(
        p for p in (env.get('GZ_SIM_RESOURCE_PATH'), env.get('SDF_PATH')) if p)
    return env


@pytest.mark.parametrize('world', ALL)
def test_world_keeps_the_contract(world):
    """Named default, with every system a vehicle needs, buoyancy naming nobody, a sea."""
    root = ET.parse(WORLDS / f'{world}.sdf').getroot()
    w = root.find('world')
    # Every world is named 'default' so /world/default/... commands work on all.
    assert w.get('name') == 'default'
    plugins = {p.get('filename') for p in w.findall('plugin')}
    assert set(REQUIRED_SYSTEMS) <= plugins, set(REQUIRED_SYSTEMS) - plugins
    assert any(f.startswith('gz-sim-waves-') for f in plugins), 'no wave source'
    buoyancy = next(p for p in w.findall('plugin')
                    if p.get('filename') == 'gz-maritime-buoyancy-system')
    assert buoyancy.find('enable_by_default').text == 'false'
    assert buoyancy.find('enable') is None, 'the world must name no vehicle'
    assert w.find('spherical_coordinates/latitude_deg') is not None
    uris = [i.find('uri').text for i in w.findall('include')]
    assert any(u == 'model://water_surface' for u in uris), 'no drawn sea'


@pytest.mark.parametrize('world', LOCAL)
def test_world_passes_the_sdf_check(world):
    """The SDF checker accepts the world with every include resolved."""
    out = subprocess.run(['gz', 'sdf', '-k', str(WORLDS / f'{world}.sdf')],
                         env=sdf_env(), capture_output=True, text=True, timeout=120)
    assert out.returncode == 0 and 'Valid' in out.stdout + out.stderr, \
        f'{world}:\n{out.stdout[-2000:]}\n{out.stderr[-2000:]}'


@pytest.mark.parametrize('world', LOCAL_TERRAIN)
def test_world_with_local_terrain_runs_headless(world):
    """The server loads the terrain and its plugins and steps the world."""
    env = dict(os.environ, GZ_PARTITION=f'test_{uuid.uuid4().hex[:8]}')
    out = subprocess.run(['gz', 'sim', '-s', '-r', '--iterations', '200',
                          str(WORLDS / f'{world}.sdf')],
                         env=env, capture_output=True, text=True, timeout=300)
    assert out.returncode == 0, f'{world} did not run:\n{out.stdout[-2000:]}\n{out.stderr[-2000:]}'
    for bad in ('Unable to find', 'Failed to load', 'Error parsing'):
        assert bad not in out.stdout + out.stderr, f'{world}: {bad} in the server output'
