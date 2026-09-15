#!/usr/bin/env python3
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
Write the RViz config for one vehicle instance and print its path.

In a simulation every TF frame carries the instance name and the description
is published in the instance's namespace, so RViz needs three things a base
config cannot know: the fixed frame <name>/base_link, the description topic
/<name>/robot_description and the RobotModel display's TF Prefix, without
which it looks the links up under their bare URDF names and draws nothing.
None of them can be given on the command line, hence this script, which
rviz.launch.xml runs before starting RViz.
"""

import argparse
import pathlib
import re
import sys

import yaml

# The same rule the spawn launch applies: a Gazebo model name, a topic
# prefix, a ROS namespace and a TF frame prefix at once.
NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


def instance_config(text, name):
    """Return the base RViz config `text` pointed at instance `name`."""
    config = yaml.safe_load(text)
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = f'{name}/base_link'
    manager['Views']['Current']['Target Frame'] = f'{name}/base_link'
    for display in manager['Displays']:
        if display.get('Class') == 'rviz_default_plugins/RobotModel':
            display['TF Prefix'] = name
            display['Description Topic']['Value'] = f'/{name}/robot_description'
    return yaml.safe_dump(config, sort_keys=False)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--name', required=True,
                    help='instance name: TF prefix, namespace and topic prefix')
    ap.add_argument('--config', required=True, type=pathlib.Path,
                    help='the base RViz config, for the unprefixed description')
    ap.add_argument('--out', required=True, type=pathlib.Path,
                    help='where to write the instance config')
    args = ap.parse_args(argv)
    if not NAME.match(args.name):
        sys.exit(f'instance_rviz: invalid instance name {args.name!r}: letters, '
                 'digits and underscores, starting with a letter')
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(instance_config(args.config.read_text(), args.name))
    print(args.out)


if __name__ == '__main__':
    main()
