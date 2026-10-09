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
The OceanCurrentPanel and WindPanel plugins in a running GUI.

A server runs a test world; the GUI runs with only the two plugins, on Qt's
offscreen platform. Each plugin logs where it read its recipe from, and the
test reads that log: plugins loaded with the GUI read their recipes from the
GUI's own state, and the wind panel in a world without a wind asks the
world's state service once, finds none, and carries on.
"""

import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

import pytest

HERE = Path(__file__).resolve().parent
CONFIG = HERE / 'gui.config'
WORLDS = HERE / 'worlds'


class Gui:
    """A server and a GUI in a partition of their own, and the GUI's log."""

    def __init__(self, world):
        env = dict(os.environ,
                   GZ_PARTITION='gz_maritime_gui_test_' + uuid.uuid4().hex,
                   QT_QPA_PLATFORM='offscreen')
        self.lines = []
        self.server = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '-v', '3', str(world)],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.gui = subprocess.Popen(
            ['gz', 'sim', '-g', '-v', '4', '--gui-config', str(CONFIG)],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.gui.stdout:
            self.lines.append(line)

    def wait_for(self, text, timeout=60.0):
        """Wait for a line with the text; return whether it came."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if any(text in line for line in self.lines):
                return True
            time.sleep(0.2)
        return False

    def has(self, text):
        """Whether a line so far has the text."""
        return any(text in line for line in self.lines)

    def stop(self):
        for proc in (self.gui, self.server):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        for proc in (self.gui, self.server):
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


@pytest.fixture
def gui():
    started = []

    def start(world):
        started.append(Gui(WORLDS / world))
        return started[-1]

    yield start
    for g in started:
        g.stop()


def test_reads_both_recipes_from_the_gui_state(gui):
    g = gui('wind_and_current.sdf')
    assert g.wait_for('Loaded plugin [OceanCurrentPanel]'), ''.join(g.lines)
    assert g.wait_for('Loaded plugin [WindPanel]'), ''.join(g.lines)
    assert g.wait_for(
        "OceanCurrentPanel: ocean current from the GUI's state"), \
        ''.join(g.lines)
    assert g.wait_for("WindPanel: wind from the GUI's state"), \
        ''.join(g.lines)
    assert not g.has('asking the world'), \
        'a plugin loaded with the GUI needs no state service'


def test_world_without_wind(gui):
    g = gui('current_only.sdf')
    assert g.wait_for(
        "OceanCurrentPanel: ocean current from the GUI's state"), \
        ''.join(g.lines)
    assert g.wait_for("WindPanel: no recipe in the GUI's state yet"), \
        ''.join(g.lines)
    # It carries on without a wind: the GUI is still up a while later.
    time.sleep(3.0)
    assert g.gui.poll() is None, ''.join(g.lines)
    assert not g.has('WindPanel: wind from')
    assert not g.has('OceanCurrentPanel: no recipe'), \
        'the current panel had its recipe and asked nothing'
