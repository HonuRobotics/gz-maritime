#!/usr/bin/env python3
# Copyright (C) 2026 Honu Robotics
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
Register entities with the world's buoyancy system from outside the model.

For models that carry no BuoyancyEnable plugin (gz_buoyancy). Each name is a
model or a model::link, as spawned:

    ros2 run kai_bringup buoyancy_enable.py blueboat::hull_displacement

The script waits for a Gazebo server, asks it for its world name, and calls
/world/<world>/buoyancy/enable once per name, retrying until each call is
accepted or --timeout seconds have passed. Calling before the spawn is fine:
the system accepts a name that matches nothing yet, and re-scans existing
links whenever a name is added.

It needs the gz-transport Python bindings, which the ROS Gazebo vendor
packages do not ship: python3-gz-transport15 for Gazebo Jetty, or
python3-gz-transport14 for Ionic, from packages.osrfoundation.org.
"""

import argparse
import functools
import sys
import time

try:
    # Gazebo Jetty and later drop the major version from module names.
    from gz.msgs.boolean_pb2 import Boolean
    from gz.msgs.empty_pb2 import Empty
    from gz.msgs.stringmsg_pb2 import StringMsg
    from gz.msgs.stringmsg_v_pb2 import StringMsg_V
    from gz.transport import Node
except ImportError:
    try:
        from gz.msgs11.boolean_pb2 import Boolean
        from gz.msgs11.empty_pb2 import Empty
        from gz.msgs11.stringmsg_pb2 import StringMsg
        from gz.msgs11.stringmsg_v_pb2 import StringMsg_V
        from gz.transport14 import Node
    except ImportError:
        Node = None

# Reply timeout for a single request [ms]. Short, because a server that is not
# up yet answers nothing, and the caller retries anyway.
REQUEST_TIMEOUT_MS = 1000

# Pause between attempts [s].
RETRY_PERIOD_S = 0.5


def world_name(node):
    """
    Ask the running server for its world name.

    Done the way ros_gz_sim create does it, rather than assuming "default",
    because the launch accepts any world file.

    :param node: gz-transport node to send the request from.
    :return: The world name, or None if no server answered.
    """
    ok, reply = node.request('/gazebo/worlds', Empty(), Empty, StringMsg_V,
                             REQUEST_TIMEOUT_MS)
    return reply.data[0] if ok and reply.data else None


def enable(node, world, name):
    """
    Ask a world's buoyancy system to enable one entity.

    :param node: gz-transport node to send the request from.
    :param world: World name.
    :param name: Model or model::link, as spawned.
    :return: True if the system accepted the name.
    """
    request = StringMsg()
    request.data = name
    ok, reply = node.request(f'/world/{world}/buoyancy/enable', request,
                             StringMsg, Boolean, REQUEST_TIMEOUT_MS)
    return ok and reply.data


def retry(attempt, deadline):
    """
    Call attempt until it returns a truthy value or the deadline passes.

    :param attempt: Callable taking no arguments.
    :param deadline: time.monotonic() value after which to give up.
    :return: The first truthy result, or None on timeout.
    """
    while True:
        result = attempt()
        if result:
            return result
        if time.monotonic() >= deadline:
            return None
        time.sleep(RETRY_PERIOD_S)


def main(argv=None):
    """Register every name given on the command line; return an exit code."""
    parser = argparse.ArgumentParser(
        description="Register entities with the world's buoyancy system "
                    '(gz_buoyancy) from outside the model.')
    parser.add_argument(
        'names', nargs='+', metavar='NAME',
        help='model or model::link, as spawned; one argument may hold '
             'several, space-separated, as a launch argument does')
    parser.add_argument(
        '--timeout', type=float, default=60.0,
        help='seconds to keep retrying before giving up (default: 60)')

    argv = sys.argv[1:] if argv is None else argv
    # Launched as a <node>, which appends ROS arguments this script has no
    # use for.
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    args = parser.parse_args(argv)

    names = [name for arg in args.names for name in arg.split()]
    if not names:
        parser.error('no names given')

    if Node is None:
        print('The gz-transport Python bindings are not installed. Install '
              'python3-gz-transport15 for Gazebo Jetty, or '
              'python3-gz-transport14 for Ionic.', file=sys.stderr)
        return 1

    node = Node()
    deadline = time.monotonic() + args.timeout
    try:
        world = retry(functools.partial(world_name, node), deadline)
        if world is None:
            print('No Gazebo server answered /gazebo/worlds within '
                  f'{args.timeout:g} s.', file=sys.stderr)
            return 1

        for name in names:
            if not retry(functools.partial(enable, node, world, name),
                         deadline):
                print(f'/world/{world}/buoyancy/enable did not accept [{name}] '
                      f'within {args.timeout:g} s. Does world [{world}] load '
                      'gz-maritime-buoyancy-system?', file=sys.stderr)
                return 1
            print(f'Enabled buoyancy for [{name}] in world [{world}].',
                  flush=True)
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    sys.exit(main())
