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
"""The validation tools: the metric, a closed loop, and the chart world."""

import json
import math
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parent.parent / 'tools'
sys.path.insert(0, str(TOOLS))

import chart_validation as cv  # noqa: E402, I100, I202
import make_chart_world as mcw  # noqa: E402, I100

# Jerlov 3C and 7C as gz_water_optics resolves them (c, veil = b / c).
WATERS = {
    '3C': (np.array([1.162, 1.228, 1.654]), np.array([0.736, 0.936, 0.907])),
    '7C': (np.array([2.257, 2.773, 3.842]), np.array([0.837, 0.916, 0.859])),
}


def to_srgb8(lin):
    lin = np.clip(lin, 0, 1)
    s = np.where(lin <= 0.0031308, 12.92 * lin, 1.055 * lin ** (1 / 2.4) - 0.055)
    return np.round(s * 255).astype(np.uint8)


def chart_image(water, rng_m=2.0, depth=5.0, gain=1.0, size=(240, 360)):
    """Draw a synthetic underwater chart: the revised model on the patches."""
    c, veil = WATERS[water]
    h, w = size
    img = np.zeros((h, w, 3))
    boxes = []
    ph, pw = h // mcw.ROWS, w // mcw.COLS
    for k, (_, rgb) in enumerate(mcw.PATCHES):
        r, col = divmod(k, mcw.COLS)
        j = np.array([mcw.linear(x) for x in rgb]) * np.exp(-0.3 * depth)
        t = np.exp(-c * rng_m)
        img[r * ph:(r + 1) * ph, col * pw:(col + 1) * pw] = gain * (
            j * t + veil * np.exp(-0.2 * depth) * (1 - t))
        boxes.append([col * pw, r * ph, (col + 1) * pw, (r + 1) * ph])
    return to_srgb8(img), boxes


def test_angular_error_basics():
    assert cv.angular_error([1, 0, 0], [1, 0, 0]) == pytest.approx(0)
    assert cv.angular_error([1, 0, 0], [0, 1, 0]) == pytest.approx(90)
    assert cv.angular_error([1, 1, 0], [1, 0, 0]) == pytest.approx(45)
    # Brightness does not count, only hue and saturation.
    assert cv.angular_error([0.2, 0.3, 0.4], [0.4, 0.6, 0.8]) == pytest.approx(0, abs=1e-4)
    assert math.isnan(cv.angular_error([0, 0, 0], [1, 0, 0]))


def test_patch_means_shrink_away_from_edges():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    img[2:8, 2:8] = 255
    img[2, 2:8] = 0  # a dark edge row the shrink must skip
    means = cv.patch_means(img, [[2, 2, 8, 8]])
    assert means[0] == pytest.approx([1, 1, 1])


def test_closed_loop_same_water_is_zero():
    real, boxes = chart_image('3C')
    sim, _ = chart_image('3C')
    assert cv.compare(real, sim, boxes)['mean'] < 0.5


def test_closed_loop_exposure_does_not_count():
    real, boxes = chart_image('3C', gain=1.0)
    sim, _ = chart_image('3C', gain=0.6)
    assert cv.compare(real, sim, boxes)['mean'] < 1.0


def test_closed_loop_other_water_is_far():
    real, boxes = chart_image('3C')
    sim, _ = chart_image('7C')
    result = cv.compare(real, sim, boxes)
    assert result['mean'] > 3.0
    assert len(result['errors']) == 24


def test_command_line(tmp_path):
    from PIL import Image
    real, boxes = chart_image('3C')
    sim, _ = chart_image('3C', gain=0.8)
    Image.fromarray(real).save(tmp_path / 'real.png')
    Image.fromarray(sim).save(tmp_path / 'sim.png')
    json.dump({'patches': [{'name': n, 'box': b} for (n, _), b in zip(mcw.PATCHES, boxes)]},
              open(tmp_path / 'chart.json', 'w'))
    out = subprocess.run([sys.executable, str(TOOLS / 'chart_validation.py'),
                          '--real', str(tmp_path / 'real.png'), '--sim', str(tmp_path / 'sim.png'),
                          '--patches', str(tmp_path / 'chart.json'),
                          '--report', str(tmp_path / 'report.png'),
                          '--json', str(tmp_path / 'result.json')],
                         capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr
    assert (tmp_path / 'report.png').is_file()
    assert json.load(open(tmp_path / 'result.json'))['mean'] < 1.0


def test_chart_world(tmp_path):
    out = tmp_path / 'chart.sdf'
    mcw.main(['--out', str(out), '--range', '2', '--depth', '5', '--jerlov', '3C',
              '--width', '640', '--height', '360'])
    assert out.is_file()
    boxes = json.load(open(tmp_path / 'chart.json'))['patches']
    assert len(boxes) == 24
    for b in boxes:
        x0, y0, x1, y1 = b['box']
        assert 0 <= x0 < x1 <= 640 and 0 <= y0 < y1 <= 360, b
    # Patch 1 top left, patch 24 bottom right, as the camera sees it.
    assert boxes[0]['box'][0] < boxes[5]['box'][0]
    assert boxes[0]['box'][1] < boxes[23]['box'][1]
    check = subprocess.run(['gz', 'sdf', '-k', str(out)], capture_output=True, text=True)
    text = check.stdout + check.stderr
    assert check.returncode == 0 and 'Valid' in text, text
