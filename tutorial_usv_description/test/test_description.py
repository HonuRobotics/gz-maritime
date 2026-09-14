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
"""Description tests: the xacro expands to a valid URDF of a boat that floats."""

from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

SHARE = Path(get_package_share_directory('tutorial_usv_description'))
XACRO = SHARE / 'urdf' / 'tutorial_usv.urdf.xacro'
WATER_DENSITY = 1025.0


@pytest.fixture(scope='module')
def urdf_text():
    """Expand the installed xacro; fail with xacro's stderr on error."""
    out = subprocess.run(['xacro', str(XACRO)], capture_output=True, text=True,
                         timeout=60)
    assert out.returncode == 0, (
        f'xacro failed ({out.returncode})\n--- stderr ---\n{out.stderr}')
    return out.stdout


@pytest.fixture(scope='module')
def robot(urdf_text):
    """Return the parsed URDF root."""
    return ET.fromstring(urdf_text)


def test_check_urdf_accepts_it(urdf_text):
    """The check_urdf tool parses the expansion and finds base_link as root."""
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as f:
        f.write(urdf_text)
        path = f.name
    out = subprocess.run(['check_urdf', path], capture_output=True, text=True,
                         timeout=60)
    assert out.returncode == 0, f'check_urdf rejected the URDF:\n{out.stderr}'
    assert 'root Link: base_link' in out.stdout, out.stdout


def test_root_is_base_link(robot):
    """base_link is the one link no joint has as its child."""
    links = {link.get('name') for link in robot.findall('link')}
    children = {joint.find('child').get('link') for joint in robot.findall('joint')}
    assert links - children == {'base_link'}


def test_propeller_joints_are_continuous_about_x(robot):
    """Each propeller hangs off base_link on a continuous joint about +x."""
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    for side in ('port', 'stbd'):
        joint = joints[f'motor_{side}_joint']
        assert joint.get('type') == 'continuous'
        assert joint.find('parent').get('link') == 'base_link'
        assert joint.find('child').get('link') == f'motor_{side}'
        axis = [float(v) for v in joint.find('axis').get('xyz').split()]
        assert axis == [1.0, 0.0, 0.0]


def test_masses_and_inertias_are_positive(robot):
    """Every link with an inertial has positive mass and principal inertias."""
    inertials = {link.get('name'): link.find('inertial')
                 for link in robot.findall('link')
                 if link.find('inertial') is not None}
    assert {'base_link', 'motor_port', 'motor_stbd'} <= set(inertials)
    for name, inertial in inertials.items():
        assert float(inertial.find('mass').get('value')) > 0, name
        inertia = inertial.find('inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            assert float(inertia.get(axis)) > 0, f'{name} {axis}'


def test_sensor_frames_are_massless_and_fixed(robot):
    """imu_link and gps_link are bare frames fixed to base_link."""
    links = {link.get('name'): link for link in robot.findall('link')}
    joints = {joint.find('child').get('link'): joint
              for joint in robot.findall('joint')}
    for frame in ('imu_link', 'gps_link'):
        assert len(links[frame]) == 0, f'{frame} should carry nothing'
        assert joints[frame].get('type') == 'fixed'
        assert joints[frame].find('parent').get('link') == 'base_link'


def test_stays_gazebo_free(urdf_text, robot):
    """
    Nothing simulator-specific: no gazebo extension, no gz attributes.

    The displacement marks and the plugins belong to tutorial_usv_gazebo, so
    this file works unchanged in RViz and on the real vehicle.
    """
    assert robot.find('gazebo') is None
    assert 'gz:' not in urdf_text
    assert '<plugin' not in urdf_text


def test_hulls_sit_one_draft_below_the_waterline(robot):
    """
    The hull boxes are placed with their bottoms one draft below base_link.

    That is what makes base_link's origin the design waterline: the Gazebo
    model floats the boat on displacement boxes of the same size and place,
    so in seawater it settles with base_link at z = 0.
    """
    mass = sum(float(m.get('value')) for m in robot.findall('link/inertial/mass'))
    base = next(link for link in robot.findall('link')
                if link.get('name') == 'base_link')
    boxes = []
    for collision in base.findall('collision'):
        size = [float(v) for v in collision.find('geometry/box').get('size').split()]
        z = float(collision.find('origin').get('xyz').split()[2])
        boxes.append((size, z))
    assert len(boxes) == 2, 'expected one contact box per hull'
    waterplane = sum(lx * ly for (lx, ly, _), _ in boxes)
    draft = mass / (WATER_DENSITY * waterplane)
    for (_, _, lz), z in boxes:
        assert z - lz / 2 == pytest.approx(-draft, abs=1e-6)
        assert draft < lz, 'waterline above the hull tops: the boat sinks'
