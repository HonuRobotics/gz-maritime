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
Produce the files of one vehicle instance for the spawn launch.

The spawn launch knows nothing about any vehicle. It needs, for one instance
name, a model to spawn and, when the vehicle has them, a bridge config and a
URDF for robot_state_publisher. This script produces those three files under
one directory from either of two sources:

  --generator CMD    a command run as `CMD --name <name> --out-dir <dir>`
                     that writes model.sdf, ros_gz_bridge.yaml and a URDF there
  --xacro FILE       a model xacro rendered with name:=<name>, plus an optional
                     --bridge FILE bridge template with @name@ where the name
                     goes and an optional --urdf FILE (xacro or plain), which
                     the model xacro receives as urdf_uri:=file://<rendered>

Whatever the source, the instance directory holds model.sdf, and when
available ros_gz_bridge.yaml and robot.urdf, with the name filled in. Any
/clock entry is dropped from the bridge config: the simulation launch bridges
the clock once for every vehicle. The launch picks the directory (by default
$ROS_HOME/kai_bringup/<name>) and passes it as --out-dir. --has-bridge and
--has-urdf print true or false for an instance already produced, so the
launch can skip what a vehicle lacks.
"""

import argparse
import pathlib
import re
import shlex
import subprocess
import sys

import yaml

TOKEN = '@name@'
# A name is a Gazebo model name, a topic prefix, a ROS namespace and a TF
# frame prefix at once, so it is held to the strictest of those.
NAME = re.compile(r'^[A-Za-z][A-Za-z0-9_]*$')


def fail(message):
    sys.exit(f'instantiate_vehicle: {message}')


def pick_urdf(directory):
    """Return the URDF a generator wrote, or None; Gazebo flavoured copies are skipped."""
    urdfs = sorted(p for p in pathlib.Path(directory).glob('*.urdf')
                   if not p.name.endswith('.gazebo.urdf'))
    return urdfs[0] if urdfs else None


def from_generator(command, name, out_dir):
    """Run `command --name name --out-dir out_dir`; return (model, bridge, urdf) paths."""
    cmd = shlex.split(command) + ['--name', name, '--out-dir', str(out_dir)]
    out = subprocess.run(cmd, capture_output=True, text=True)
    if out.returncode != 0:
        fail(f'generator failed ({out.returncode}): {shlex.join(cmd)}\n{out.stderr}')
    model = out_dir / 'model.sdf'
    if not model.is_file():
        fail(f'generator wrote no model.sdf into {out_dir}: {shlex.join(cmd)}')
    bridge = out_dir / 'ros_gz_bridge.yaml'
    return model, bridge if bridge.is_file() else None, pick_urdf(out_dir)


def render(path, out, name, *args):
    """Write `path` to `out`: run through xacro with name:=name when it is a xacro."""
    path = pathlib.Path(path)
    if not path.is_file():
        fail(f'no such file: {path}')
    if path.suffix == '.xacro':
        cmd = ['xacro', str(path), f'name:={name}', *args]
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            fail(f'xacro failed: {shlex.join(cmd)}\n{result.stderr}')
        out.write_text(result.stdout)
    else:
        out.write_text(path.read_text())
    return out


def from_xacro(model, bridge, urdf, name, out_dir):
    """Render the model (and URDF) with xacro into out_dir; return (model, bridge, urdf)."""
    rendered_urdf = render(urdf, out_dir / 'robot.urdf', name) if urdf else None
    args = [f'urdf_uri:=file://{rendered_urdf}'] if rendered_urdf else []
    rendered_model = render(model, out_dir / 'model.sdf', name, *args)
    bridge = pathlib.Path(bridge) if bridge else None
    if bridge is not None and not bridge.is_file():
        fail(f'no such file: {bridge}')
    return rendered_model, bridge, rendered_urdf


def without_clock(entries):
    """Return the bridge entries minus the clock, which the simulation bridges once."""
    return [e for e in entries
            if '/clock' not in (e.get('ros_topic_name'), e.get('gz_topic_name'))]


def instantiate(name, out_dir, model, bridge=None, urdf=None):
    """Write the instance files for `name` into out_dir; the bridge gets the name filled in."""
    out_dir = pathlib.Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for target, source in (('model.sdf', model), ('robot.urdf', urdf)):
        if source is None:
            (out_dir / target).unlink(missing_ok=True)
        elif source.resolve() != (out_dir / target).resolve():
            (out_dir / target).write_text(source.read_text())
    if bridge is None:
        (out_dir / 'ros_gz_bridge.yaml').unlink(missing_ok=True)
    else:
        entries = yaml.safe_load(bridge.read_text().replace(TOKEN, name)) or []
        (out_dir / 'ros_gz_bridge.yaml').write_text(
            yaml.safe_dump(without_clock(entries), sort_keys=False))
    return out_dir


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--name', required=True,
                    help='instance name: model name, topic prefix, namespace and TF prefix')
    source = ap.add_argument_group('source, one of')
    source.add_argument('--generator', default='',
                        help='command run as `<command> --name <name> --out-dir <dir>`')
    source.add_argument('--xacro', default='',
                        help='model xacro rendered with name:=<name> '
                             '(and urdf_uri:= when a URDF is given)')
    ap.add_argument('--bridge', default='', help='bridge config with @name@ placeholders')
    ap.add_argument('--urdf', default='',
                    help='URDF (xacro or plain) for robot_state_publisher, with --xacro')
    ap.add_argument('--out-dir', required=True, help='directory to write the instance into')
    query = ap.add_mutually_exclusive_group()
    query.add_argument('--has-bridge', action='store_true',
                       help='print whether the instance already produced has a bridge config')
    query.add_argument('--has-urdf', action='store_true',
                       help='print whether the instance already produced has a URDF')
    args = ap.parse_args(argv)

    if not NAME.match(args.name):
        fail(f'invalid instance name {args.name!r}: letters, digits and underscores, '
             'starting with a letter')
    out_dir = pathlib.Path(args.out_dir)
    if args.has_bridge or args.has_urdf:
        wanted = 'ros_gz_bridge.yaml' if args.has_bridge else 'robot.urdf'
        print('true' if (out_dir / wanted).is_file() else 'false')
        return

    if bool(args.generator) == bool(args.xacro):
        fail('give exactly one of --generator or --xacro')
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.generator:
        model, bridge, urdf = from_generator(args.generator, args.name, out_dir)
    else:
        model, bridge, urdf = from_xacro(args.xacro, args.bridge, args.urdf, args.name, out_dir)
    instantiate(args.name, out_dir, model, bridge, urdf)
    have = ['model.sdf'] + (['ros_gz_bridge.yaml'] if bridge else []) \
        + (['robot.urdf'] if urdf else [])
    print(f'wrote {", ".join(have)} for {args.name} to {out_dir}')


if __name__ == '__main__':
    main()
