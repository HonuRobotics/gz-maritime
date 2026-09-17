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
The multi vehicle demo: one ocean with one vehicle of each type on it.

The simulation launch once, then the spawn launch once per vehicle, each
through the vehicle's own generator: a BlueBoat, a BlueROV2 six metres
ahead of it and one metre down, and an X500 on a landing pad. Each vehicle
gets its topics under its name, its bridge and state publisher in its
namespace and its own TF prefix; the clock is bridged once.

`world:=` picks the world, as in every launch here, and with it where the
vehicles go, from the table below: the Sydney Regatta lake by default, or
the open water world. Needs bluerobotics_models and holybro_models built
in the workspace. A vehicle of another type joins a running demo the same
way: one more spawn launch with its generator or its files and a new name.

    ros2 launch kai_bringup multi_vehicle_demo.launch.py
    ros2 launch kai_bringup multi_vehicle_demo.launch.py world:=open_water.sdf gazebo_gui:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare

# Per world, by file name: the pose of each vehicle (x, y, z, yaw). The
# boat sits at the waterline, the ROV a metre under, the X500 on the pad of
# that world, which is 1 m above the water; z = 1.25 drops it onto its
# landing gear.
POSES = {
    'sydney_regatta.sdf': {
        'blueboat': (-532, 162, 0.05, 1.0),
        'bluerov2': (-528.8, 167.0, -1, 1.0),
        'x500': (-540, 168, 1.25, 0),
    },
    'open_water.sdf': {
        'blueboat': (0, 0, 0.05, 0),
        'bluerov2': (6, 0, -1, 0),
        'x500': (8, -8, 1.25, 0),
    },
}

# The generator of each vehicle: the spawn launch runs it with the name.
GENERATORS = {
    'blueboat': ('blueboat_gazebo', 'blueboat_description', 'blueboat'),
    'bluerov2': ('bluerov2_gazebo', 'bluerov2_description', 'bluerov2'),
    'x500': ('x500_gazebo', 'x500_description', 'x500'),
}


def generator(gazebo_pkg, description_pkg, vehicle):
    """Return the configure_vehicle.py command of a vehicle, as a substitution list."""
    return [FindPackagePrefix(gazebo_pkg), f'/lib/{gazebo_pkg}/configure_vehicle.py --config ',
            FindPackageShare(description_pkg), f'/config/{vehicle}.yaml']


def demo(context):
    """Return the simulation launch and one spawn launch per vehicle, for the world."""
    world = LaunchConfiguration('world').perform(context)
    if world not in POSES:
        raise ValueError(f'the demo knows where to put its vehicles in {sorted(POSES)}, '
                         f'not in {world}')
    poses = POSES[world]
    use_composition = LaunchConfiguration('use_composition')
    actions = [IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [FindPackageShare('kai_bringup'), '/launch/simulation.launch.xml']),
        launch_arguments={
            'world': world,
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'use_composition': use_composition,
        }.items())]
    for name, (gazebo_pkg, description_pkg, vehicle) in GENERATORS.items():
        x, y, z, yaw = poses[name]
        # Each spawn in its own group, so what one instance derives from
        # its name does not leak into the next.
        actions.append(GroupAction([IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                [FindPackageShare('kai_bringup'), '/launch/spawn_vehicle.launch.xml']),
            launch_arguments={
                'name': name,
                'generator': generator(gazebo_pkg, description_pkg, vehicle),
                'world': '',
                'x': str(x), 'y': str(y), 'z': str(z), 'yaw': str(yaw),
                'use_composition': use_composition,
            }.items())], scoped=True))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='sydney_regatta.sdf',
            description='World SDF file name (resolved via GZ_SIM_RESOURCE_PATH); '
                        f'the demo places its vehicles in {sorted(POSES)}.'),
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true', description='Launch the Gazebo GUI'),
        DeclareLaunchArgument(
            'use_composition', default_value='true',
            description='Run Gazebo, the bridges and the state publishers in one container'),
        OpaqueFunction(function=demo),
    ])
