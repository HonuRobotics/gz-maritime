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
"""Tests of the installed default model: it parses, and its references hold."""

import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest
import yaml

GZ_SHARE = Path(get_package_share_directory('tutorial_usv_gazebo'))
DESC_SHARE = Path(get_package_share_directory('tutorial_usv_description'))
MODEL_DIR = GZ_SHARE / 'models' / 'tutorial_usv'
BRIDGE_YAML = GZ_SHARE / 'config' / 'ros_gz_bridge.yaml'
NAME = 'tutorial_usv'
GZ_NS = '{http://gazebosim.org/schema}'


def gz_sdf_print(path):
    """
    Return `gz sdf -p` of a file, parsed.

    The gz sdf CLI resolves model:// URIs through SDF_PATH, not
    GZ_SIM_RESOURCE_PATH, so the one is handed the other.
    """
    env = dict(os.environ)
    env['SDF_PATH'] = os.pathsep.join(
        p for p in (env.get('GZ_SIM_RESOURCE_PATH'), env.get('SDF_PATH')) if p)
    out = subprocess.run(['gz', 'sdf', '-p', str(path)], env=env,
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, f'gz sdf -p {path} failed:\n{out.stderr}'
    return ET.fromstring(out.stdout)


@pytest.fixture(scope='module')
def model():
    """Return the installed model.sdf, fully resolved: URDF merged in."""
    return gz_sdf_print(MODEL_DIR / 'model.sdf').find('model')


@pytest.fixture(scope='module')
def raw():
    """Return the installed model.sdf as xacro wrote it."""
    return ET.parse(MODEL_DIR / 'model.sdf').getroot()


@pytest.fixture(scope='module')
def converted_urdf():
    """Return the model's URDF as Gazebo converts it on its own."""
    return gz_sdf_print(MODEL_DIR / 'tutorial_usv.urdf').find('model')


def plugins(model, filename):
    """Return the model's plugin elements with the given filename."""
    return [p for p in model.findall('plugin') if p.get('filename') == filename]


def test_model_directory_is_self_contained():
    """model://tutorial_usv holds the model, its config and the merged URDF."""
    for name in ('model.sdf', 'model.config', 'tutorial_usv.urdf'):
        assert (MODEL_DIR / name).is_file(), name
    raw = ET.parse(MODEL_DIR / 'model.sdf').getroot()
    assert raw.find('model/include/uri').text == 'model://tutorial_usv/tutorial_usv.urdf'
    installed = (DESC_SHARE / 'urdf' / 'tutorial_usv.urdf').read_text()
    assert (MODEL_DIR / 'tutorial_usv.urdf').read_text() == installed, \
        'the model merges a stale copy of the description'


def test_displacement_link_marks_its_pontoons(raw, model):
    """
    The boat floats by two marked boxes on a link of its own.

    The marks are what the world reads, so they must survive xacro; the link
    is fixed to base_link so physics sees one body; the boxes match the
    URDF's hulls in size and place; and they are out of contact.
    """
    displacement = next(li for li in raw.iter('link')
                        if li.get('name') == 'hull_displacement')
    marked = [c for c in displacement.findall('collision')
              if c.get(f'{GZ_NS}buoyancy') == 'true']
    assert sorted(c.get('name') for c in marked) == ['pontoon_port', 'pontoon_stbd']
    for collision in marked:
        assert int(collision.find('surface/contact/collide_bitmask').text, 16) == 0
    joint = next(j for j in model.findall('joint')
                 if j.get('name') == 'hull_displacement_joint')
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').text == 'base_link'
    urdf = ET.parse(MODEL_DIR / 'tutorial_usv.urdf').getroot()
    base = next(li for li in urdf.findall('link') if li.get('name') == 'base_link')
    hulls = {c.get('name'): c for c in base.findall('collision')}
    for side in ('port', 'stbd'):
        pontoon = next(c for c in marked if c.get('name') == f'pontoon_{side}')
        hull = hulls[f'hull_{side}']
        assert pontoon.find('geometry/box/size').text.split() == \
            hull.find('geometry/box').get('size').split()
        assert pontoon.find('pose').text.split()[:3] == \
            hull.find('origin').get('xyz').split()


def test_urdf_hulls_are_not_marked(raw):
    """Only the displacement link floats: the URDF's contact hulls carry no mark."""
    unmarked = [c.get('name') for c in raw.iter('collision')
                if c.get(f'{GZ_NS}buoyancy') != 'true']
    assert unmarked == []   # the URDF's collisions are merged in by Gazebo, not xacro
    urdf = ET.parse(MODEL_DIR / 'tutorial_usv.urdf').getroot()
    assert 'gz:' not in ET.tostring(urdf, encoding='unicode')


def test_two_counter_rotating_thrusters(model):
    """One Thruster per propeller joint, opposite coefficients, seawater."""
    thrusters = {t.find('joint_name').text: t
                 for t in plugins(model, 'gz-sim-thruster-system')}
    assert set(thrusters) == {'motor_port_joint', 'motor_stbd_joint'}
    port, stbd = thrusters['motor_port_joint'], thrusters['motor_stbd_joint']
    assert float(port.find('thrust_coefficient').text) == \
        -float(stbd.find('thrust_coefficient').text)
    for side, thruster in (('port', port), ('stbd', stbd)):
        assert float(thruster.find('fluid_density').text) == 1025.0
        assert thruster.find('topic').text == f'{NAME}/motor_{side}/thrust'
        assert (thruster.find('namespace').text or '') == ''


def test_thruster_diameter_matches_the_propeller(model):
    """The Thruster diameter is the one the URDF gives the propeller."""
    urdf = ET.parse(DESC_SHARE / 'urdf' / 'tutorial_usv.urdf').getroot()
    for thruster in plugins(model, 'gz-sim-thruster-system'):
        link_name = thruster.find('joint_name').text.removesuffix('_joint')
        link = next(li for li in urdf.findall('link') if li.get('name') == link_name)
        radius = float(link.find('visual/geometry/cylinder').get('radius'))
        assert float(thruster.find('propeller_diameter').text) == \
            pytest.approx(2 * radius)


def test_hydrodynamics_on_base_link(model):
    """One Hydrodynamics plugin, on the link the displacement is fixed to."""
    hydro = plugins(model, 'gz-sim-hydrodynamics-system')
    assert len(hydro) == 1
    assert hydro[0].find('link_name').text == 'base_link'


def test_three_sensors_at_urdf_frames(model):
    """IMU, magnetometer and NavSat, posed at URDF frames, publishing them prefixed."""
    sensors = {}
    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            sensors[sensor.get('type')] = (link, sensor)
    assert set(sensors) == {'imu', 'magnetometer', 'navsat'}
    expected_frame = {'imu': 'imu_link', 'magnetometer': 'imu_link',
                      'navsat': 'gps_link'}
    for kind, (link, sensor) in sensors.items():
        assert link.find('pose').get('relative_to') == expected_frame[kind]
        assert sensor.find('frame_id').text == f'{NAME}/{expected_frame[kind]}'
        assert sensor.find('topic').text == f'{NAME}/{kind}'


def test_references_survive_the_urdf_conversion(model, converted_urdf):
    """
    What the plugins and sensors name exists after Gazebo converts the URDF.

    The conversion lumps fixed joints away: imu_link and gps_link stop being
    links and survive only as frames, so references are checked against the
    converted model rather than the URDF.
    """
    links = {li.get('name') for li in converted_urdf.findall('link')}
    joints = {j.get('name') for j in converted_urdf.findall('joint')}
    frames = {f.get('name') for f in converted_urdf.findall('frame')}
    assert {'imu_link', 'gps_link'} <= frames
    assert {'imu_link', 'gps_link'}.isdisjoint(links)
    assert {'motor_port_joint', 'motor_stbd_joint'} <= joints
    for plugin in model.findall('plugin'):
        for ref in plugin.findall('joint_name'):
            assert ref.text in joints, ref.text
        for ref in plugin.findall('link_name'):
            assert ref.text in links, ref.text
    sensor_frames = {li.find('pose').get('relative_to')
                     for li in model.findall('link') if li.find('sensor') is not None}
    assert sensor_frames <= frames
    displacement = next(li for li in model.findall('link')
                        if li.get('name') == 'hull_displacement')
    assert displacement.find('pose').get('relative_to') == 'base_link'


def test_sensor_frame_ids_are_prefixed_urdf_links(raw):
    """
    Every frame_id is <name>/<URDF link>.

    That is the frame robot_state_publisher publishes when it runs with
    frame_prefix set to the instance name, so the messages name a frame TF
    carries.
    """
    urdf = ET.parse(DESC_SHARE / 'urdf' / 'tutorial_usv.urdf').getroot()
    urdf_links = {li.get('name') for li in urdf.findall('link')}
    frame_ids = {f.text for f in raw.iter('frame_id')}
    assert frame_ids
    for frame_id in frame_ids:
        prefix, _, link = frame_id.partition('/')
        assert prefix == NAME and link in urdf_links, frame_id


def test_bridge_config_matches_the_model_topics(model):
    """Every model gz topic is bridged, every bridged gz topic exists, no clock."""
    topics = {s.find('topic').text for li in model.findall('link')
              for s in li.findall('sensor')}
    for thruster in plugins(model, 'gz-sim-thruster-system'):
        topic = thruster.find('topic').text
        topics |= {topic, f'{topic}/ang_vel'}
    for jsp in plugins(model, 'gz-sim-joint-state-publisher-system'):
        topics.add(jsp.find('topic').text)
    model_topics = {'/' + t.lstrip('/') for t in topics}
    bridged = {e['gz_topic_name'] for e in yaml.safe_load(BRIDGE_YAML.read_text())}
    assert bridged == model_topics
