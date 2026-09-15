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
Generation tests: the vehicle's three sources render right for any name.

kai_bringup's instantiate_vehicle.py renders this package's model xacro, URDF
xacro and bridge template for an instance name, which is what
spawn_vehicle.launch.xml does at launch. Two instances are rendered here and
checked: the URDF is valid and Gazebo-free, the model floats by marked
collisions, drives two normalized thrusters and carries three sensors at URDF
frames, every topic and frame id carries the name, and the bridge lists
exactly the model's topics. The installed default instance, model://custom_usv,
is checked as well.
"""

import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
import pytest
import yaml

SHARE = Path(get_package_share_directory('kai_custom_vehicle'))
MODEL_XACRO = SHARE / 'models' / 'custom_usv' / 'model.sdf.xacro'
URDF_XACRO = SHARE / 'urdf' / 'custom_usv.urdf.xacro'
BRIDGE_TEMPLATE = SHARE / 'config' / 'ros_gz_bridge.yaml.in'
DEFAULT_DIR = SHARE / 'models' / 'custom_usv'
INSTANTIATE = (Path(get_package_prefix('kai_bringup'))
               / 'lib' / 'kai_bringup' / 'instantiate_vehicle.py')
NAMES = ('boat_a', 'boat_b')
GZ_NS = '{http://gazebosim.org/schema}'
THRUSTER = 'gz-maritime-thruster-system'
WATER_DENSITY = 1025.0


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


def plugins(model, filename):
    """Return the model's plugin elements with the given filename."""
    return [p for p in model.findall('plugin') if p.get('filename') == filename]


class Instance:
    """The files rendered for one name, parsed every way the tests need."""

    def __init__(self, name):
        self.name = name
        self.dir = Path(tempfile.mkdtemp(prefix=f'{name}_'))
        out = subprocess.run(
            [str(INSTANTIATE), '--name', name, '--out-dir', str(self.dir),
             '--xacro', str(MODEL_XACRO), '--bridge', str(BRIDGE_TEMPLATE),
             '--urdf', str(URDF_XACRO)],
            capture_output=True, text=True, timeout=120)
        assert out.returncode == 0, f'instantiate failed:\n{out.stdout}\n{out.stderr}'
        # model.sdf as xacro wrote it (the marks live here) ...
        self.raw = ET.parse(self.dir / 'model.sdf').getroot()
        # ... and fully resolved by Gazebo, the URDF merged in.
        self.model = gz_sdf_print(self.dir / 'model.sdf').find('model')
        self.urdf_text = (self.dir / 'robot.urdf').read_text()
        self.urdf = ET.fromstring(self.urdf_text)
        # The URDF as Gazebo converts it on its own: fixed joints lumped away.
        self.converted_urdf = gz_sdf_print(self.dir / 'robot.urdf').find('model')
        self.bridge = yaml.safe_load((self.dir / 'ros_gz_bridge.yaml').read_text())


@pytest.fixture(scope='module')
def instances():
    """Render the three sources for two names; return name -> Instance."""
    return {name: Instance(name) for name in NAMES}


@pytest.fixture(scope='module')
def boat(instances):
    """One rendered instance, for the checks that hold for every name."""
    return instances['boat_a']


def test_default_instance_is_installed_and_self_contained():
    """model://custom_usv holds the model, its config, the merged URDF and the bridge."""
    for name in ('model.sdf', 'model.config', 'custom_usv.urdf', 'ros_gz_bridge.yaml'):
        assert (DEFAULT_DIR / name).is_file(), name
    raw = ET.parse(DEFAULT_DIR / 'model.sdf').getroot()
    assert raw.find('model').get('name') == 'custom_usv'
    assert raw.find('model/include/uri').text == 'model://custom_usv/custom_usv.urdf'
    config = ET.parse(DEFAULT_DIR / 'model.config').getroot()
    assert config.find('name').text == 'custom_usv'
    # Resolving through model:// proves the resource path hook and the URDF copy.
    assert gz_sdf_print(DEFAULT_DIR / 'model.sdf').find('model').get('name') == 'custom_usv'
    bridged = yaml.safe_load((DEFAULT_DIR / 'ros_gz_bridge.yaml').read_text())
    assert all(e['gz_topic_name'].startswith('/custom_usv/') for e in bridged)


def test_urdf_is_valid_with_base_link_as_root(boat):
    """check_urdf accepts the rendered URDF, and base_link is the one root."""
    out = subprocess.run(['check_urdf', str(boat.dir / 'robot.urdf')],
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, f'check_urdf rejected the URDF:\n{out.stderr}'
    assert 'root Link: base_link' in out.stdout, out.stdout
    links = {link.get('name') for link in boat.urdf.findall('link')}
    children = {joint.find('child').get('link') for joint in boat.urdf.findall('joint')}
    assert links - children == {'base_link'}


def test_urdf_stays_gazebo_free_and_nameless(boat):
    """
    Nothing simulator-specific and no instance name in the URDF.

    The marks and the plugins belong to the simulation model, so this file
    works unchanged in RViz and on the real vehicle; the name comes from the
    spawn launch (frame_prefix), so one URDF serves every instance.
    """
    assert boat.urdf.find('gazebo') is None
    assert 'gz:' not in boat.urdf_text
    assert '<plugin' not in boat.urdf_text
    assert boat.name not in boat.urdf_text


def test_propeller_joints_are_continuous_about_x(boat):
    """Each propeller hangs off base_link on a continuous joint about +x."""
    joints = {joint.get('name'): joint for joint in boat.urdf.findall('joint')}
    for side in ('port', 'stbd'):
        joint = joints[f'motor_{side}_joint']
        assert joint.get('type') == 'continuous'
        assert joint.find('parent').get('link') == 'base_link'
        assert joint.find('child').get('link') == f'motor_{side}'
        assert [float(v) for v in joint.find('axis').get('xyz').split()] == [1.0, 0.0, 0.0]


def test_masses_and_inertias_are_positive(boat):
    """Every link with an inertial has positive mass and principal inertias."""
    inertials = {link.get('name'): link.find('inertial')
                 for link in boat.urdf.findall('link')
                 if link.find('inertial') is not None}
    assert {'base_link', 'motor_port', 'motor_stbd'} <= set(inertials)
    for name, inertial in inertials.items():
        assert float(inertial.find('mass').get('value')) > 0, name
        for axis in ('ixx', 'iyy', 'izz'):
            assert float(inertial.find('inertia').get(axis)) > 0, f'{name} {axis}'


def test_hulls_sit_one_draft_below_the_waterline(boat):
    """
    The hull boxes are placed with their bottoms one draft below base_link.

    That is what makes base_link's origin the design waterline: the model
    floats the boat on displacement boxes of the same size and place, so in
    seawater it settles with base_link at z = 0.
    """
    mass = sum(float(m.get('value')) for m in boat.urdf.findall('link/inertial/mass'))
    base = next(li for li in boat.urdf.findall('link') if li.get('name') == 'base_link')
    boxes = []
    for collision in base.findall('collision'):
        size = [float(v) for v in collision.find('geometry/box').get('size').split()]
        z = float(collision.find('origin').get('xyz').split()[2])
        boxes.append((size, z))
    assert len(boxes) == 2, 'expected one contact box per hull'
    draft = mass / (WATER_DENSITY * sum(lx * ly for (lx, ly, _), _ in boxes))
    for (_, _, lz), z in boxes:
        assert z - lz / 2 == pytest.approx(-draft, abs=1e-6)
        assert draft < lz, 'waterline above the hull tops: the boat sinks'


def test_displacement_link_marks_its_pontoons(boat):
    """
    The boat floats by two marked boxes on a link of its own.

    The marks are what the world reads, so they must survive xacro; the link
    is fixed to base_link so physics sees one body; the boxes match the
    URDF's hulls in size and place; and they are out of contact. The URDF's
    own hulls carry no mark.
    """
    displacement = next(li for li in boat.raw.iter('link')
                        if li.get('name') == 'hull_displacement')
    marked = [c for c in displacement.findall('collision')
              if c.get(f'{GZ_NS}buoyancy') == 'true']
    assert sorted(c.get('name') for c in marked) == ['pontoon_port', 'pontoon_stbd']
    for collision in marked:
        assert int(collision.find('surface/contact/collide_bitmask').text, 16) == 0
    unmarked = [c.get('name') for c in boat.raw.iter('collision') if c not in marked]
    assert unmarked == []   # the URDF's collisions are merged in by Gazebo, not xacro
    joint = next(j for j in boat.model.findall('joint')
                 if j.get('name') == 'hull_displacement_joint')
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').text == 'base_link'
    base = next(li for li in boat.urdf.findall('link') if li.get('name') == 'base_link')
    hulls = {c.get('name'): c for c in base.findall('collision')}
    for side in ('port', 'stbd'):
        pontoon = next(c for c in marked if c.get('name') == f'pontoon_{side}')
        hull = hulls[f'hull_{side}']
        assert pontoon.find('geometry/box/size').text.split() == \
            hull.find('geometry/box').get('size').split()
        assert pontoon.find('pose').text.split()[:3] == \
            hull.find('origin').get('xyz').split()


def test_two_normalized_counter_rotating_thrusters(boat):
    """
    One gz-maritime Thruster per propeller joint, in normalized mode.

    Commands in [-1, 1] on /<name>/motor_<side>/cmd, opposite coefficients
    for the two propellers, seawater, and the propeller diameter the URDF
    gives.
    """
    thrusters = {t.find('joint_name').text: t for t in plugins(boat.model, THRUSTER)}
    assert set(thrusters) == {'motor_port_joint', 'motor_stbd_joint'}
    assert not plugins(boat.model, 'gz-sim-thruster-system')
    port, stbd = thrusters['motor_port_joint'], thrusters['motor_stbd_joint']
    assert float(port.find('thrust_coefficient').text) == \
        -float(stbd.find('thrust_coefficient').text)
    for side, thruster in (('port', port), ('stbd', stbd)):
        assert thruster.find('use_normalized_cmd').text == 'true'
        assert float(thruster.find('max_thrust_cmd').text) > 0
        assert float(thruster.find('min_thrust_cmd').text) < 0
        assert float(thruster.find('fluid_density').text) == WATER_DENSITY
        assert thruster.find('topic').text == f'{boat.name}/motor_{side}/cmd'
        assert (thruster.find('namespace').text or '') == ''
        link = next(li for li in boat.urdf.findall('link')
                    if li.get('name') == f'motor_{side}')
        radius = float(link.find('visual/geometry/cylinder').get('radius'))
        assert float(thruster.find('propeller_diameter').text) == pytest.approx(2 * radius)


def test_hydrodynamics_on_base_link(boat):
    """One Hydrodynamics plugin, on the link the displacement is fixed to."""
    hydro = plugins(boat.model, 'gz-sim-hydrodynamics-system')
    assert len(hydro) == 1
    assert hydro[0].find('link_name').text == 'base_link'


def test_three_sensors_at_urdf_frames(boat):
    """
    IMU, magnetometer and NavSat, posed at URDF frames, publishing them prefixed.

    Every frame_id is <name>/<URDF link>: the frame robot_state_publisher
    publishes when it runs with frame_prefix set to the instance name.
    """
    sensors = {}
    for link in boat.model.findall('link'):
        for sensor in link.findall('sensor'):
            sensors[sensor.get('type')] = (link, sensor)
    assert set(sensors) == {'imu', 'magnetometer', 'navsat'}
    expected_frame = {'imu': 'imu_link', 'magnetometer': 'imu_link', 'navsat': 'gps_link'}
    urdf_links = {li.get('name') for li in boat.urdf.findall('link')}
    for kind, (link, sensor) in sensors.items():
        assert link.find('pose').get('relative_to') == expected_frame[kind]
        assert sensor.find('frame_id').text == f'{boat.name}/{expected_frame[kind]}'
        assert expected_frame[kind] in urdf_links
        assert sensor.find('topic').text == f'{boat.name}/{kind}'


def test_references_survive_the_urdf_conversion(boat):
    """
    What the plugins and sensors name exists after Gazebo converts the URDF.

    The conversion lumps fixed joints away: imu_link and gps_link stop being
    links and survive only as frames, so references are checked against the
    converted model rather than the URDF.
    """
    links = {li.get('name') for li in boat.converted_urdf.findall('link')}
    joints = {j.get('name') for j in boat.converted_urdf.findall('joint')}
    frames = {f.get('name') for f in boat.converted_urdf.findall('frame')}
    assert {'imu_link', 'gps_link'} <= frames
    assert {'imu_link', 'gps_link'}.isdisjoint(links)
    assert {'motor_port_joint', 'motor_stbd_joint'} <= joints
    for plugin in boat.model.findall('plugin'):
        for ref in plugin.findall('joint_name'):
            assert ref.text in joints, ref.text
        for ref in plugin.findall('link_name'):
            assert ref.text in links, ref.text
    sensor_frames = {li.find('pose').get('relative_to')
                     for li in boat.model.findall('link') if li.find('sensor') is not None}
    assert sensor_frames <= frames
    displacement = next(li for li in boat.model.findall('link')
                        if li.get('name') == 'hull_displacement')
    assert displacement.find('pose').get('relative_to') == 'base_link'


def test_bridge_config_matches_the_model_topics(boat):
    """Every model gz topic is bridged, every bridged gz topic exists, no clock."""
    topics = {s.find('topic').text for li in boat.model.findall('link')
              for s in li.findall('sensor')}
    for thruster in plugins(boat.model, THRUSTER):
        topic = thruster.find('topic').text
        topics |= {topic, f'{topic}/ang_vel'}
    for jsp in plugins(boat.model, 'gz-sim-joint-state-publisher-system'):
        topics.add(jsp.find('topic').text)
    model_topics = {'/' + t.lstrip('/') for t in topics}
    bridged = {e['gz_topic_name'] for e in boat.bridge}
    assert bridged == model_topics
    assert '/clock' not in {e['ros_topic_name'] for e in boat.bridge}


def test_every_instance_carries_its_own_name_and_nothing_shared(instances):
    """
    Model name, every topic and every frame id follow the name; nothing is shared.

    Two instances of the same vehicle in one simulation must not hear each
    other's commands nor confuse each other's frames.
    """
    seen = {}
    for name, inst in instances.items():
        assert inst.model.get('name') == name
        assert all(t.text.startswith(f'{name}/') for t in inst.raw.iter('topic')), name
        assert all(f.text.startswith(f'{name}/') for f in inst.raw.iter('frame_id')), name
        ros_topics = {e['ros_topic_name'] for e in inst.bridge}
        assert ros_topics and all(t.startswith(f'/{name}/') for t in ros_topics), name
        seen[name] = ros_topics
    assert seen['boat_a'].isdisjoint(seen['boat_b'])
