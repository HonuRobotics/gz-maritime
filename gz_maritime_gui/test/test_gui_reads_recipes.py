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
The OceanCurrentPanel, WindPanel, ScenarioPanel and AcousticsPanel plugins in a running GUI.

A server runs a test world; the GUI runs with only these plugins, on Qt's
offscreen platform. Each plugin logs where it read its recipe from, and the
test reads that log: plugins loaded with the GUI read their recipes from the
GUI's own state, and the wind panel in a world without a wind asks the
world's state service once, finds none, and carries on. The acoustics
panel, drawing everything, reads the pingers from the GUI's state and
hears the receivers the acoustics system lists.

The 3D view cannot run offscreen, so MarkerManager cannot either: a small
program, marker_recorder, serves its array service in its place and prints
what it is sent, and the test reads what the acoustics panel drew: the
reach circle's radius, the rings and the ray of a reading, and that they
are redrawn at once on a paused world. The menu cannot be clicked
offscreen, so a test plugin, MenuLoader, adds the acoustics panel late the
way the menu does.
"""

import math
import os
from pathlib import Path
import re
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

    def __init__(self, world, config=CONFIG):
        env = dict(os.environ,
                   GZ_PARTITION='gz_maritime_gui_test_' + uuid.uuid4().hex,
                   QT_QPA_PLATFORM='offscreen')
        self.env = env
        self.lines = []
        self.markers = []
        # In MarkerManager's place, before the GUI draws anything.
        self.recorder = subprocess.Popen(
            [os.environ['MARKER_RECORDER']],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        threading.Thread(target=self._read, daemon=True,
                         args=(self.recorder, self.markers)).start()
        self.server = subprocess.Popen(
            ['gz', 'sim', '-s', '-r', '-v', '3', str(world)],
            env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)
        self.gui = subprocess.Popen(
            ['gz', 'sim', '-g', '-v', '4', '--gui-config', str(config)],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, start_new_session=True)
        self.reader = threading.Thread(target=self._read, daemon=True,
                                       args=(self.gui, self.lines))
        self.reader.start()

    @staticmethod
    def _read(proc, lines):
        for line in proc.stdout:
            lines.append(line)

    def drawn(self, ns):
        """
        Return the markers a drawing has on the scene, by id.

        Each is (points, min x, max x, min y, max y), as last sent and not
        deleted since.
        """
        scene = {}
        for line in list(self.markers):
            m = re.match(rf'marker {re.escape(ns)} (\d+) (add|delete)(.*)', line)
            if not m:
                continue
            if m.group(2) == 'delete':
                scene.pop(int(m.group(1)), None)
            else:
                scene[int(m.group(1))] = tuple(float(v) for v in m.group(3).split())
        return scene

    def wait_drawn(self, ns, timeout=60.0):
        """Wait for a drawing to have a marker on the scene; return its markers."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            scene = self.drawn(ns)
            if scene:
                return scene
            time.sleep(0.2)
        return {}

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

    def service(self, name, reqtype, req):
        """Call a world service with the gz tool; return whether it said yes."""
        out = subprocess.run(
            ['gz', 'service', '-s', name, '--reqtype', reqtype,
             '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', req],
            env=self.env, capture_output=True, text=True, timeout=30)
        return 'data: true' in out.stdout

    def publish(self, topic, msgtype, msg):
        """Publish one message with the gz tool."""
        subprocess.run(['gz', 'topic', '-t', topic, '-m', msgtype, '-p', msg],
                       env=self.env, check=True, timeout=30)

    def stop(self):
        procs = (self.gui, self.server, self.recorder)
        for proc in procs:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
        for proc in procs:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()


@pytest.fixture
def gui():
    started = []

    def start(world, config=CONFIG):
        started.append(Gui(WORLDS / world, config))
        return started[-1]

    yield start
    for g in started:
        g.stop()


def test_reads_both_recipes_from_the_gui_state(gui):
    g = gui('wind_and_current.sdf')
    assert g.wait_for('Loaded plugin [OceanCurrentPanel]'), ''.join(g.lines)
    assert g.wait_for('Loaded plugin [WindPanel]'), ''.join(g.lines)
    assert g.wait_for('Loaded plugin [ScenarioPanel]'), ''.join(g.lines)
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


def test_acoustics_from_the_gui_state(gui):
    g = gui('acoustics.sdf')
    assert g.wait_for('Loaded plugin [AcousticsPanel]'), ''.join(g.lines)
    assert g.wait_for("AcousticsPanel: pingers from the GUI's state: 1"), \
        ''.join(g.lines)
    assert g.wait_for('AcousticsPanel: receivers listed by the acoustics system: 1'), \
        ''.join(g.lines)
    # On the receiver's plane, 1 m down, once it is found.
    assert g.wait_for('AcousticsPanel: drawing at the depth of listener'), \
        ''.join(g.lines)
    # Its QML loads clean and it keeps running.
    time.sleep(3.0)
    assert g.gui.poll() is None, ''.join(g.lines)
    assert not g.has('AcousticsPanel.qml:'), ''.join(g.lines)
    # It asks nobody for the world's state.
    assert not any('AcousticsPanel' in line and 'asking' in line
                   for line in g.lines), ''.join(g.lines)


NS = 'gz_maritime_gui/acoustics'


def config_with(tmp_path, extra):
    """Return the GUI config with elements added to the acoustics panel's."""
    text = CONFIG.read_text().replace('<plane>receiver</plane>',
                                      '<plane>receiver</plane>' + extra)
    path = tmp_path / 'gui.config'
    path.write_text(text)
    return path


def test_acoustics_plane_follows_the_receiver_named(gui, tmp_path):
    """
    Follow the receiver <plane_receiver> names, and still after a retune.

    The second receiver is named: the plane follows it, and still does after
    a receiver is retuned, which replaces the receivers' list. The combo
    box's row lives in QML, which no offscreen test reads; this covers the
    panel's side of it.
    """
    g = gui('two_receivers.sdf', config_with(
        tmp_path, '<plane_receiver>/diver/acoustic_receiver</plane_receiver>'))
    assert g.wait_for('AcousticsPanel: receivers listed by the acoustics system: 2'), \
        ''.join(g.lines)
    assert g.wait_for('AcousticsPanel: drawing at the depth of diver'), \
        ''.join(g.lines)
    assert not g.has('drawing at the depth of listener'), ''.join(g.lines)
    # Retune the first receiver: the list is replaced, the plane stays.
    g.publish('/listener/acoustic_receiver/set', 'gz.msgs.Param',
              'params {key: "frequency" value {type: DOUBLE double_value: 25000}}')
    time.sleep(3.0)
    assert g.gui.poll() is None, ''.join(g.lines)
    assert not g.has('drawing at the depth of listener'), ''.join(g.lines)
    assert not g.has('AcousticsPanel.qml:'), ''.join(g.lines)


def test_acoustics_plane_receiver_not_listed(gui, tmp_path):
    """A <plane_receiver> nobody lists is warned about once, and the water drawn on."""
    g = gui('two_receivers.sdf', config_with(
        tmp_path, '<plane_receiver>/nobody/acoustic_receiver</plane_receiver>'))
    assert g.wait_for('AcousticsPanel: receivers listed by the acoustics system: 2'), \
        ''.join(g.lines)
    assert g.wait_for('<plane_receiver> /nobody/acoustic_receiver is not a receiver'), \
        ''.join(g.lines)
    time.sleep(2.0)
    assert not g.has('drawing at the depth of'), ''.join(g.lines)
    assert sum('is not a receiver the acoustics system lists' in line
               for line in g.lines) == 1, 'warned once'
    assert not g.has('AcousticsPanel.qml:'), ''.join(g.lines)


def test_acoustics_draws_what_it_shows(gui):
    """
    The acoustics panel's markers, as MarkerManager would get them.

    In acoustics.sdf the pinger is 1.5 m under the origin with a 100 m
    reach, and the receiver 20 m east of it, 1 m down.
    """
    g = gui('acoustics.sdf')
    assert g.wait_for('Loaded plugin [AcousticsPanel]'), ''.join(g.lines)

    # The plane drawn on is the receiver's, 1 m down, once it is found;
    # until then the water.
    assert g.wait_for('AcousticsPanel: drawing at the depth of listener'), \
        ''.join(g.lines)
    time.sleep(1.0)

    # The reach: one circle about the origin, where a 100 m sphere from
    # 1.5 m down cuts that plane, half a metre above the pinger.
    reach = g.wait_drawn(NS + '/reach')
    assert len(reach) == 1, g.markers
    points, min_x, max_x, min_y, max_y = reach[1]
    radius = math.sqrt(100.0 ** 2 - 0.5 ** 2)
    assert points == 2 * 96
    assert max_x == pytest.approx(radius, abs=0.01)
    assert min_x == pytest.approx(-radius, abs=0.01)
    assert max_y == pytest.approx(radius, abs=0.01)
    assert min_y == pytest.approx(-radius, abs=0.01)

    # The pings: rings about the origin, inside the reach.
    rings = g.wait_drawn(NS + '/pings')
    assert rings, g.markers
    for points, min_x, max_x, min_y, max_y in rings.values():
        assert points % (2 * 64) == 0
        assert max_x == pytest.approx(-min_x, abs=0.01)
        assert max_y == pytest.approx(max_x, abs=0.01)
        assert 0.0 < max_x <= radius

    # A reading: a 30 m ray from the receiver at (20, 0), towards the
    # pinger to its west and half a metre below it, 20 m away.
    rays = g.wait_drawn(NS + '/readings')
    assert len(rays) == 1, g.markers
    points, min_x, max_x, min_y, max_y = next(iter(rays.values()))
    assert points == 2
    assert max_x == pytest.approx(20.0, abs=0.01)
    assert min_x == pytest.approx(20.0 - 30.0 * math.cos(math.atan2(0.5, 20.0)), abs=0.05)
    assert min_y == pytest.approx(0.0, abs=0.05)
    assert max_y == pytest.approx(0.0, abs=0.05)

    # The circle stays; it is not sent again while nothing changes.
    sent = sum(1 for line in g.markers if line.startswith(f'marker {NS}/reach '))
    time.sleep(3.0)
    assert sum(1 for line in g.markers
               if line.startswith(f'marker {NS}/reach ')) == sent, g.markers


def wait_until(check, timeout=30.0):
    """Wait for a check to pass; return whether it did."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if check():
            return True
        time.sleep(0.2)
    return False


def centre(marker):
    """Return the middle of a marker's box, (x, y)."""
    _, min_x, max_x, min_y, max_y = marker
    return (min_x + max_x) / 2.0, (min_y + max_y) / 2.0


def test_acoustics_redraws_while_paused(gui):
    """
    A paused world never reaches the timed redraw.

    A buoy dragged and a pinger switched off while paused must still show
    at once: the reach circle and the rings move with the buoy, and the
    circle turns dashed and the rings go when the pinger is off.
    """
    g = gui('acoustics.sdf')
    assert g.wait_for('Loaded plugin [AcousticsPanel]'), ''.join(g.lines)
    reach = g.wait_drawn(NS + '/reach')
    assert reach, g.markers
    assert g.wait_drawn(NS + '/pings'), g.markers
    solid = reach[1][0]

    assert g.service('/world/default/control', 'gz.msgs.WorldControl', 'pause: true')
    # Dragged to (10, 5) while paused: the circle and the rings follow.
    assert g.service('/world/default/set_pose', 'gz.msgs.Pose',
                     'name: "buoy" position { x: 10 y: 5 z: 0 }')

    def moved(ns):
        scene = g.drawn(ns)
        return bool(scene) and all(
            centre(m) == (pytest.approx(10.0, abs=0.05), pytest.approx(5.0, abs=0.05))
            for m in scene.values())
    assert wait_until(lambda: moved(NS + '/reach')), g.drawn(NS + '/reach')
    assert wait_until(lambda: moved(NS + '/pings')), g.drawn(NS + '/pings')

    # Switched off while paused: the circle is drawn dashed, the rings go.
    # A one-shot publish can go out before discovery finds the subscriber,
    # so it is sent until it shows; switching off twice changes nothing.
    def switched_off():
        g.publish('/world/default/acoustics/pinger/set', 'gz.msgs.Param',
                  'params { key: "active" value { type: BOOLEAN bool_value: false } }')
        return g.drawn(NS + '/reach').get(1, (solid,))[0] != solid
    assert wait_until(switched_off), g.drawn(NS + '/reach')
    assert wait_until(lambda: not g.drawn(NS + '/pings')), g.drawn(NS + '/pings')
    assert moved(NS + '/reach'), g.drawn(NS + '/reach')


def test_acoustics_added_from_the_menu(gui):
    """
    A panel added from the plugin menu, after the GUI has its state.

    It finds the pinger, from the acoustics system's list if the GUI's state
    does not have it, and the receiver, asks nobody, and draws nothing
    until asked: the menu gives it no configuration.
    """
    g = gui('acoustics.sdf', HERE / 'gui_menu.config')
    assert g.wait_for('MenuLoader: adding [AcousticsPanel]'), ''.join(g.lines)
    assert g.wait_for('Loaded plugin [AcousticsPanel]'), ''.join(g.lines)
    assert g.wait_for('AcousticsPanel: pingers from'), ''.join(g.lines)
    line = next(ln for ln in g.lines if 'AcousticsPanel: pingers from' in ln)
    assert re.search(r'pingers from [^:]*: 1\b', line), line
    assert g.wait_for('AcousticsPanel: receivers listed by the acoustics system: 1'), \
        ''.join(g.lines)
    time.sleep(3.0)
    assert g.gui.poll() is None, ''.join(g.lines)
    assert not g.has('AcousticsPanel.qml:'), ''.join(g.lines)
    assert not any('AcousticsPanel' in ln and 'asking' in ln
                   for ln in g.lines), ''.join(g.lines)
    assert not any(ln.startswith(f'marker {NS}/') for ln in g.markers), g.markers


def test_acoustics_in_a_world_without_pingers(gui):
    """Every world but the demo: receivers at most, no pinger, and nothing asked."""
    g = gui('receiver_only.sdf')
    assert g.wait_for('Loaded plugin [AcousticsPanel]'), ''.join(g.lines)
    assert g.wait_for('AcousticsPanel: receivers listed by the acoustics system: 1'), \
        ''.join(g.lines)
    time.sleep(3.0)
    assert g.gui.poll() is None, ''.join(g.lines)
    assert not g.has('AcousticsPanel: pingers from'), ''.join(g.lines)
    # The wind and current panels ask, in a world without them; this one
    # does not: no pinger is the normal case.
    assert not any('AcousticsPanel' in line and 'asking' in line
                   for line in g.lines), ''.join(g.lines)
    # Nothing to draw: nothing drawn.
    assert not any(line.startswith(f'marker {NS}/') for line in g.markers), g.markers


def test_acoustics_in_3d(gui):
    g = gui('acoustics.sdf', HERE / 'gui_3d.config')
    assert g.wait_for("AcousticsPanel: pingers from the GUI's state: 1"), \
        ''.join(g.lines)
    # Drawing the spheres, it keeps running, and its QML loads clean.
    time.sleep(3.0)
    assert g.gui.poll() is None, ''.join(g.lines)
    assert not g.has('AcousticsPanel.qml:'), ''.join(g.lines)
    # The reach as a sphere and the pings as shells, cut at the water: both
    # are drawn, and reach no further than the 100 m reach.
    reach = g.wait_drawn(NS + '/reach')
    assert reach, g.markers
    shells = g.wait_drawn(NS + '/pings')
    assert shells, g.markers
    for points, min_x, max_x, min_y, max_y in list(reach.values()) + list(shells.values()):
        assert points > 0
        assert -100.01 <= min_x and max_x <= 100.01
        assert -100.01 <= min_y and max_y <= 100.01
