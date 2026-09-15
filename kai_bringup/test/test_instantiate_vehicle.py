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
"""instantiate_vehicle.py: the instance files for a name, from files or a generator."""

from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix
import yaml

TOOL = Path(get_package_prefix('kai_bringup')) / 'lib' / 'kai_bringup' / 'instantiate_vehicle.py'

MODEL_XACRO = """\
<?xml version="1.0"?>
<sdf version="1.12" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:arg name="name" default="usv"/>
  <xacro:arg name="urdf_uri" default=""/>
  <model name="$(arg name)">
    <include merge="true"><uri>$(arg urdf_uri)</uri></include>
    <plugin name="p" filename="f"><topic>$(arg name)/motor/cmd</topic></plugin>
  </model>
</sdf>
"""
BRIDGE = [
    {'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
     'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'gz.msgs.Clock',
     'direction': 'GZ_TO_ROS'},
    {'ros_topic_name': '/@name@/motor/cmd', 'gz_topic_name': '/@name@/motor/cmd',
     'ros_type_name': 'std_msgs/msg/Float64', 'gz_type_name': 'gz.msgs.Double',
     'direction': 'ROS_TO_GZ'},
]
URDF_XACRO = """\
<?xml version="1.0"?>
<robot name="usv" xmlns:xacro="http://ros.org/wiki/xacro">
  <xacro:property name="w" value="${0.2 + 0.3}"/>
  <link name="base_link"><visual><geometry><box size="1 ${w} 0.3"/></geometry></visual></link>
</robot>
"""


def run(*args, check=True):
    out = subprocess.run([str(TOOL), *args], capture_output=True, text=True, timeout=120)
    if check:
        assert out.returncode == 0, out.stderr
    return out


def source_files(directory):
    model = directory / 'model.sdf.xacro'
    model.write_text(MODEL_XACRO)
    bridge = directory / 'ros_gz_bridge.yaml.in'
    bridge.write_text(yaml.safe_dump(BRIDGE))
    urdf = directory / 'usv.urdf.xacro'
    urdf.write_text(URDF_XACRO)
    return model, bridge, urdf


def test_xacro_sources_render_under_the_name_and_the_bridge_loses_the_clock(tmp_path):
    """The model and URDF xacros render for the name, the bridge is filled in, no clock."""
    model, bridge, urdf = source_files(tmp_path)
    out = tmp_path / 'boat_a'
    run('--name', 'boat_a', '--xacro', str(model), '--bridge', str(bridge),
        '--urdf', str(urdf), '--out-dir', str(out))
    sdf = ET.parse(out / 'model.sdf').getroot()
    assert sdf.find('model').get('name') == 'boat_a'
    assert sdf.find('model/plugin/topic').text == 'boat_a/motor/cmd'
    # The rendered URDF is what the model merges and what the state publisher reads.
    assert sdf.find('model/include/uri').text == f'file://{out / "robot.urdf"}'
    robot = ET.parse(out / 'robot.urdf').getroot()
    assert robot.find('link/visual/geometry/box').get('size') == '1 0.5 0.3'
    entries = yaml.safe_load((out / 'ros_gz_bridge.yaml').read_text())
    assert [e['ros_topic_name'] for e in entries] == ['/boat_a/motor/cmd']
    assert entries[0]['gz_topic_name'] == '/boat_a/motor/cmd'


def test_model_alone_is_enough(tmp_path):
    """A vehicle without a bridge or a URDF yields only model.sdf, and the queries say so."""
    model, _, _ = source_files(tmp_path)
    out = tmp_path / 'boat_a'
    run('--name', 'boat_a', '--xacro', str(model), '--out-dir', str(out))
    assert sorted(p.name for p in out.iterdir()) == ['model.sdf']
    assert run('--name', 'boat_a', '--out-dir', str(out), '--has-bridge').stdout.strip() == 'false'
    assert run('--name', 'boat_a', '--out-dir', str(out), '--has-urdf').stdout.strip() == 'false'
    # A second run with more files, then a third with fewer, leaves nothing stale.
    _, bridge, urdf = source_files(tmp_path)
    run('--name', 'boat_a', '--xacro', str(model), '--bridge', str(bridge),
        '--urdf', str(urdf), '--out-dir', str(out))
    assert run('--name', 'boat_a', '--out-dir', str(out), '--has-urdf').stdout.strip() == 'true'
    run('--name', 'boat_a', '--xacro', str(model), '--out-dir', str(out))
    assert sorted(p.name for p in out.iterdir()) == ['model.sdf']


def fake_generator(directory):
    """Write a generator following the contract: `--name N --out-dir D` fills D."""
    script = directory / 'generate.py'
    script.write_text(f"""#!{sys.executable}
import pathlib, sys
name = sys.argv[sys.argv.index('--name') + 1]
out = pathlib.Path(sys.argv[sys.argv.index('--out-dir') + 1])
out.mkdir(parents=True, exist_ok=True)
(out / 'model.sdf').write_text('<model name="%s"/>' % name)
(out / 'ros_gz_bridge.yaml').write_text({yaml.safe_dump(BRIDGE)!r}.replace('@name@', name))
(out / 'boat.gazebo.urdf').write_text('gazebo flavour, not for TF')
(out / 'boat.urdf').write_text('<robot name="%s"/>' % name)
""")
    script.chmod(0o755)
    return script


def test_generator_mode_takes_what_the_generator_wrote(tmp_path):
    """The generator's directory is read by convention; the plain URDF is the one kept."""
    script = fake_generator(tmp_path)
    instance = tmp_path / 'boat_b'
    run('--name', 'boat_b', '--generator', str(script), '--out-dir', str(instance))
    assert (instance / 'model.sdf').read_text() == '<model name="boat_b"/>'
    assert (instance / 'robot.urdf').read_text() == '<robot name="boat_b"/>'
    entries = yaml.safe_load((instance / 'ros_gz_bridge.yaml').read_text())
    assert [e['ros_topic_name'] for e in entries] == ['/boat_b/motor/cmd']


def test_bad_input_is_refused(tmp_path):
    """A name that is not a ROS name, two sources, or a missing file stop with a reason."""
    model, _, _ = source_files(tmp_path)
    out = run('--name', 'boat-b', '--xacro', str(model), '--out-dir', str(tmp_path / 'v'),
              check=False)
    assert out.returncode != 0 and 'invalid instance name' in out.stderr
    out = run('--name', 'boat_b', '--xacro', str(model), '--generator', 'x',
              '--out-dir', str(tmp_path / 'v'), check=False)
    assert out.returncode != 0 and 'exactly one' in out.stderr
    out = run('--name', 'boat_b', '--xacro', str(tmp_path / 'missing.sdf.xacro'),
              '--out-dir', str(tmp_path / 'v'), check=False)
    assert out.returncode != 0 and 'no such file' in out.stderr
