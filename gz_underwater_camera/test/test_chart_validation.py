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


def save_chart(tmp_path, name, image, boxes):
    """Write an image and its boxes; return their paths."""
    from PIL import Image
    Image.fromarray(image).save(tmp_path / f'{name}.png')
    json.dump({'patches': [{'name': n, 'box': b} for (n, _), b in zip(mcw.PATCHES, boxes)]},
              open(tmp_path / f'{name}.json', 'w'))
    return str(tmp_path / f'{name}.png'), str(tmp_path / f'{name}.json')


def run_tool(*args):
    return subprocess.run([sys.executable, str(TOOLS / 'chart_validation.py'), *args],
                          capture_output=True, text=True)


def test_command_line(tmp_path):
    real, boxes = chart_image('3C')
    sim, _ = chart_image('3C', gain=0.8)
    real_png, chart = save_chart(tmp_path, 'real', real, boxes)
    sim_png, _ = save_chart(tmp_path, 'sim', sim, boxes)
    out = run_tool('--real', real_png, '--sim', f'3C={sim_png}', '--patches', chart,
                   '--report', str(tmp_path / 'report.png'),
                   '--json', str(tmp_path / 'result.json'))
    assert out.returncode == 0, out.stdout + out.stderr
    assert (tmp_path / 'report.png').is_file()
    result = json.load(open(tmp_path / 'result.json'))
    assert result['closest'] == ['3C']
    assert result['waters']['3C']['mean'] < 1.0


def test_nearest_water_comes_first(tmp_path):
    # The real chart in 3C, simulated in 7C and 3C: 3C is the closest.
    real, boxes = chart_image('3C')
    real_png, chart = save_chart(tmp_path, 'real', real, boxes)
    sims = []
    for water in ('7C', '3C'):
        png, _ = save_chart(tmp_path, water, chart_image(water)[0], boxes)
        sims += ['--sim', f'{water}={png}']
    out = run_tool('--real', real_png, *sims, '--patches', chart,
                   '--json', str(tmp_path / 'result.json'))
    assert out.returncode == 0, out.stdout + out.stderr
    assert json.load(open(tmp_path / 'result.json'))['closest'] == ['3C', '7C']
    assert 'closest: 3C, 7C' in out.stdout


def test_threshold_is_optional(tmp_path):
    real, boxes = chart_image('3C')
    real_png, chart = save_chart(tmp_path, 'real', real, boxes)
    sim_png, _ = save_chart(tmp_path, 'sim', chart_image('7C')[0], boxes)
    assert run_tool('--real', real_png, '--sim', sim_png, '--patches', chart).returncode == 0
    out = run_tool('--real', real_png, '--sim', sim_png, '--patches', chart,
                   '--threshold', '1')
    assert out.returncode == 1


def test_own_boxes_for_each_image(tmp_path):
    # The photo and the render need not be the same size or line up.
    real, real_boxes = chart_image('3C', size=(240, 360))
    sim, sim_boxes = chart_image('3C', size=(360, 600))
    real_png, real_chart = save_chart(tmp_path, 'real', real, real_boxes)
    sim_png, sim_chart = save_chart(tmp_path, 'sim', sim, sim_boxes)
    out = run_tool('--real', real_png, '--real-patches', real_chart,
                   '--sim', sim_png, '--sim-patches', sim_chart,
                   '--json', str(tmp_path / 'result.json'))
    assert out.returncode == 0, out.stdout + out.stderr
    assert json.load(open(tmp_path / 'result.json'))['waters']['sim']['mean'] < 0.5


def test_dark_patches_are_counted(tmp_path):
    real, boxes = chart_image('3C')
    sim = real.copy()
    for x0, y0, x1, y1 in boxes[:6]:
        sim[y0:y1, x0:x1] = 0
    result = cv.compare(real, sim, boxes)
    assert result['dark'] == 6
    real_png, chart = save_chart(tmp_path, 'real', real, boxes)
    sim_png, _ = save_chart(tmp_path, 'sim', sim, boxes)
    out = run_tool('--real', real_png, '--sim', sim_png, '--patches', chart,
                   '--json', str(tmp_path / 'result.json'))
    assert out.returncode == 1, 'six dark patches, more than the four allowed'
    text = open(tmp_path / 'result.json').read()
    assert 'NaN' not in text
    json.loads(text, parse_constant=lambda c: pytest.fail(f'{c} in the JSON'))
    assert run_tool('--real', real_png, '--sim', sim_png, '--patches', chart,
                    '--max-dark', '6').returncode == 0


def test_scale_comes_from_the_data_type():
    # A nearly black 8-bit frame is not read as [0, 1].
    dark = np.ones((4, 4, 3), dtype=np.uint8)
    assert cv.srgb_to_linear(dark).max() < 0.001
    assert cv.srgb_to_linear(np.ones((4, 4, 3))).max() == pytest.approx(1.0)


def test_box_too_small_or_outside():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match='too small'):
        cv.patch_means(img, [[4, 2, 4, 8]])
    with pytest.raises(ValueError, match='outside'):
        cv.patch_means(img, [[5, 5, 20, 20]])


def test_luminance_ratio_tells_exposure_from_water():
    real, boxes = chart_image('3C')
    exposure = cv.compare(real, chart_image('3C', gain=0.6)[0], boxes)
    assert exposure['luminance_ratio'] == pytest.approx(0.6, rel=0.05)
    water = cv.compare(real, chart_image('7C')[0], boxes)
    assert water['luminance_spread'] > 3 * exposure['luminance_spread']


def test_save_frame_reads_camera_images():
    image_pb2 = pytest.importorskip('gz.msgs.image_pb2')
    import save_frame
    msg = image_pb2.Image()
    msg.width, msg.height, msg.step = 2, 1, 8  # padded rows
    msg.pixel_format_type = msg.DESCRIPTOR.fields_by_name[
        'pixel_format_type'].enum_type.values_by_name['RGB_INT8'].number
    msg.data = bytes([10, 20, 30, 40, 50, 60, 0, 0])
    assert save_frame.to_array(msg).tolist() == [[[10, 20, 30], [40, 50, 60]]]


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


def test_chart_world_options(tmp_path):
    out = tmp_path / 'run.1' / 'chart.sdf'
    out.parent.mkdir()
    mcw.main(['--out', str(out), '--jerlov', '5c', '--visibility', '4'])
    assert (tmp_path / 'run.1' / 'chart.json').is_file()
    text = out.read_text()
    assert '<jerlov>5C</jerlov>' in text and '<visibility>4.0</visibility>' in text
    for bad in (['--jerlov', '4C'], ['--visibility', '0']):
        with pytest.raises(SystemExit):
            mcw.main(['--out', str(tmp_path / 'x.sdf'), *bad])
