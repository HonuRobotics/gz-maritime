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
Tests of configure_vehicle.py: one name drives every artifact of an instance.

Two instances generated from two names must differ in nothing but the name,
and the name must reach the model, every topic, every sensor frame and the
bridge config, or two boats in one simulation would share something.
"""

import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
import pytest
import yaml

SCRIPT = (Path(get_package_prefix('tutorial_usv_gazebo'))
          / 'lib' / 'tutorial_usv_gazebo' / 'configure_vehicle.py')
GZ_NS = '{http://gazebosim.org/schema}'


def generate(name):
    """Run the installed generator for a name into a fresh directory."""
    out_dir = Path(tempfile.mkdtemp(prefix=f'{name}_'))
    result = subprocess.run([str(SCRIPT), '--name', name, '--out-dir', str(out_dir)],
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, (
        f'configure_vehicle.py failed ({result.returncode})\n{result.stderr}')
    return out_dir


@pytest.fixture(scope='module')
def boats():
    """Two instances, boat_a and boat_b."""
    return {name: generate(name) for name in ('boat_a', 'boat_b')}


def test_writes_every_artifact(boats):
    """A model directory, its bridge config and the URDF it merges."""
    for out_dir in boats.values():
        for name in ('model.sdf', 'model.config', 'tutorial_usv.urdf',
                     'ros_gz_bridge.yaml', 'tutorial_usv.rviz'):
            assert (out_dir / name).is_file(), name


def test_model_parses_with_its_urdf_merged(boats):
    """Parse each model with gz sdf: the file:// URDF resolves and the name holds."""
    for name, out_dir in boats.items():
        out = subprocess.run(['gz', 'sdf', '-p', str(out_dir / 'model.sdf')],
                             capture_output=True, text=True, timeout=60)
        assert out.returncode == 0, f'gz sdf -p failed for {name}:\n{out.stderr}'
        model = ET.fromstring(out.stdout).find('model')
        assert model.get('name') == name
        assert {li.get('name') for li in model.findall('link')} >= {
            'base_link', 'motor_port', 'motor_stbd', 'hull_displacement'}


def test_name_reaches_model_topics_and_frames(boats):
    """The name is the model name, every topic prefix and every frame prefix."""
    for name, out_dir in boats.items():
        raw = ET.parse(out_dir / 'model.sdf').getroot()
        model = raw.find('model')
        assert model.get('name') == name
        assert model.find('include/uri').text == f'file://{out_dir}/tutorial_usv.urdf'
        topics = [t.text for t in raw.iter('topic')]
        assert topics and all(t.startswith(f'{name}/') for t in topics), topics
        frames = [f.text for f in raw.iter('frame_id')]
        assert frames and all(f.startswith(f'{name}/') for f in frames), frames
        config = ET.parse(out_dir / 'model.config').getroot()
        assert config.find('name').text == name


def test_marks_survive_xacro(boats):
    """
    The pontoons come out of xacro still marked as buoyancy geometry.

    Exactly the two displacement boxes carry gz:buoyancy="true" and a zero
    collide_bitmask; the hulls from the URDF are plain contact geometry.
    """
    for out_dir in boats.values():
        raw = ET.parse(out_dir / 'model.sdf').getroot()
        marked = [c for c in raw.iter('collision')
                  if c.get(f'{GZ_NS}buoyancy') == 'true']
        assert sorted(c.get('name') for c in marked) == ['pontoon_port', 'pontoon_stbd']
        for collision in marked:
            bitmask = collision.find('surface/contact/collide_bitmask')
            assert bitmask is not None and int(bitmask.text, 16) == 0
        displacement = next(li for li in raw.iter('link')
                            if li.get('name') == 'hull_displacement')
        assert len(displacement.findall('collision')) == 2


def test_bridge_config_carries_the_name_and_no_clock(boats):
    """Every bridged topic is under /<name>, and /clock is left to the world."""
    for name, out_dir in boats.items():
        entries = yaml.safe_load((out_dir / 'ros_gz_bridge.yaml').read_text())
        assert entries
        for entry in entries:
            assert entry['gz_topic_name'].startswith(f'/{name}/'), entry
            assert entry['ros_topic_name'].startswith(f'/{name}/'), entry
        assert '/clock' not in {e['gz_topic_name'] for e in entries}


def test_rviz_config_points_at_the_instance(boats):
    """
    The RViz config looks the robot up where this instance publishes it.

    Every TF frame carries the name, so the fixed frame and the RobotModel
    display's TF Prefix must too, and the description is on the instance's
    namespaced topic.
    """
    for name, out_dir in boats.items():
        config = yaml.safe_load((out_dir / 'tutorial_usv.rviz').read_text())
        manager = config['Visualization Manager']
        assert manager['Global Options']['Fixed Frame'] == f'{name}/base_link'
        robots = [d for d in manager['Displays']
                  if d['Class'] == 'rviz_default_plugins/RobotModel']
        assert len(robots) == 1
        assert robots[0]['TF Prefix'] == name
        assert robots[0]['Description Topic']['Value'] == f'/{name}/robot_description'


def test_two_instances_share_nothing_but_the_urdf(boats):
    """boat_a and boat_b differ only where the name appears."""
    a, b = boats['boat_a'], boats['boat_b']
    assert (a / 'tutorial_usv.urdf').read_text() == (b / 'tutorial_usv.urdf').read_text()
    for name in ('model.sdf', 'ros_gz_bridge.yaml', 'model.config', 'tutorial_usv.rviz'):
        # The directory first: its own name contains the instance name.
        text_a = (a / name).read_text().replace(str(a), 'DIR').replace('boat_a', 'NAME')
        text_b = (b / name).read_text().replace(str(b), 'DIR').replace('boat_b', 'NAME')
        assert text_a == text_b, name
        assert 'boat_b' not in (a / name).read_text()


def test_rejects_a_name_that_is_not_a_namespace():
    """A name has to work as a model name, a ROS namespace and a TF prefix."""
    result = subprocess.run([str(SCRIPT), '--name', 'boat-a', '--out-dir',
                             tempfile.mkdtemp()],
                            capture_output=True, text=True, timeout=60)
    assert result.returncode != 0
    assert 'invalid instance name' in result.stderr


def test_cache_prints_a_per_name_directory(tmp_path):
    """--cache writes under $ROS_HOME/tutorial_usv_gazebo/<name> and prints it."""
    env = dict(os.environ, ROS_HOME=str(tmp_path))
    result = subprocess.run([str(SCRIPT), '--name', 'boat_c', '--cache'],
                            capture_output=True, text=True, timeout=120, env=env)
    assert result.returncode == 0, result.stderr
    assert result.stdout == str(tmp_path / 'tutorial_usv_gazebo' / 'boat_c')
    assert (Path(result.stdout) / 'model.sdf').is_file()
