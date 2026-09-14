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
Regenerate the RViz screenshot used in the documentation.

Runs the tutorial USV simulation headless, starts RViz on it through
tutorial_usv_gazebo's rviz.launch.xml on a virtual X display (Xvfb, software
GL), and photographs the RViz window with xwd. The virtual display is what
makes the picture reproducible: nothing else is on it, so the window is
captured whole, panels included, whatever is on the real screen.

Needs a sourced workspace with gz-maritime built, plus Xvfb, xwininfo, xwd
and ffmpeg (packages xvfb, x11-utils, x11-apps, ffmpeg on Ubuntu).

    python3 docs/_tools/rviz_figure.py
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

from PIL import Image

DOCS = Path(__file__).resolve().parent.parent
OUT = DOCS / 'getting-started' / 'images' / 'tutorial-usv-rviz.jpg'
DISPLAY = ':99'
SIM_SETTLE = 20      # seconds for Gazebo, the bridge and TF to come up
RVIZ_SETTLE = 40     # seconds for RViz to start and render under software GL


def start(cmd, env, log):
    """Start a process group so the whole launch tree can be stopped at once."""
    return subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)


def stop(proc, sig=signal.SIGINT):
    """Signal a process group and wait for it."""
    try:
        os.killpg(proc.pid, sig)
        proc.wait(timeout=20)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        os.killpg(proc.pid, signal.SIGKILL)


def rviz_window(env):
    """Return the X id of the RViz main window on the virtual display."""
    tree = subprocess.run(['xwininfo', '-root', '-tree'], env=env,
                          capture_output=True, text=True).stdout
    for line in tree.splitlines():
        if '- RViz"' in line:
            return line.split()[0]
    sys.exit('no RViz window on the virtual display; see the log')


def main():
    for tool in ('Xvfb', 'xwininfo', 'xwd', 'ffmpeg'):
        if subprocess.run(['which', tool], capture_output=True).returncode != 0:
            sys.exit(f'{tool} not found')
    # A private partition and domain, so a running simulation is left alone.
    env = dict(os.environ,
               GZ_PARTITION=f'docs_rviz_{uuid.uuid4().hex[:8]}',
               ROS_DOMAIN_ID=str(int(uuid.uuid4().hex[:2], 16) % 100 + 1))
    vdisplay = dict(env, DISPLAY=DISPLAY, LIBGL_ALWAYS_SOFTWARE='1')

    with tempfile.TemporaryDirectory() as tmp, \
            open(Path(tmp) / 'log.txt', 'w') as log:
        xvfb = subprocess.Popen(['Xvfb', DISPLAY, '-screen', '0', '1280x900x24'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        sim = start(['ros2', 'launch', 'tutorial_usv_gazebo', 'sim.launch.xml',
                     'gazebo_gui:=false'], env, log)
        try:
            time.sleep(SIM_SETTLE)
            rviz = start(['ros2', 'launch', 'tutorial_usv_gazebo', 'rviz.launch.xml'],
                         vdisplay, log)
            try:
                time.sleep(RVIZ_SETTLE)
                window = rviz_window(vdisplay)
                xwd = Path(tmp) / 'rviz.xwd'
                png = Path(tmp) / 'rviz.png'
                subprocess.run(['xwd', '-id', window, '-out', str(xwd)],
                               env=vdisplay, check=True)
                subprocess.run(['ffmpeg', '-loglevel', 'error', '-y', '-i',
                                str(xwd), str(png)], check=True)
                Image.open(png).convert('RGB').save(OUT, quality=88, optimize=True)
            finally:
                stop(rviz)
        finally:
            stop(sim)
            xvfb.terminate()
            xvfb.wait(timeout=10)
    print(OUT.relative_to(DOCS))


if __name__ == '__main__':
    main()
