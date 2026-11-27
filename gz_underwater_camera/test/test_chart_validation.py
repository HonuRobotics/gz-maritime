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
import re
import subprocess
import sys

import numpy as np
import pytest

TOOLS = Path(__file__).resolve().parent.parent / 'tools'
sys.path.insert(0, str(TOOLS))

import chart_validation as cv  # noqa: E402, I100, I202
import make_chart_world as mcw  # noqa: E402, I100

JERLOV_CC = (Path(__file__).resolve().parents[2] / 'gz_water_optics' / 'src'
             / 'Jerlov.cc')


def to_srgb8(lin):
    lin = np.clip(lin, 0, 1)
    s = np.where(lin <= 0.0031308, 12.92 * lin, 1.055 * lin ** (1 / 2.4) - 0.055)
    return np.round(s * 255).astype(np.uint8)


def chart_image(water, rng_m=0.5, depth=2.0, gain=1.0, size=(240, 360),
                light=1.0):
    """
    Draw a synthetic underwater chart, as the underwater camera does.

    The revised model, per channel, with the water as gz_water_optics
    resolves it from the Jerlov table: the patch lit by daylight faded
    to its depth by K_d, over the range by c, plus the veil of that depth.
    """
    c, veil, kd = (np.array(v) for v in mcw.resolve(water))
    h, w = size
    img = np.zeros((h, w, 3))
    boxes = []
    ph, pw = h // mcw.ROWS, w // mcw.COLS
    day = np.exp(-kd * depth)
    t = np.exp(-c * rng_m)
    for k, (_, rgb) in enumerate(mcw.PATCHES):
        r, col = divmod(k, mcw.COLS)
        j = light * np.array([mcw.linear(x) for x in rgb]) * day
        img[r * ph:(r + 1) * ph, col * pw:(col + 1) * pw] = gain * (
            j * t + veil * day * (1 - t))
        boxes.append([col * pw, r * ph, (col + 1) * pw, (r + 1) * ph])
    return to_srgb8(img), boxes


def test_table_is_gz_water_optics_own():
    # The tools' copy of the Jerlov table is the one the camera uses.
    text = JERLOV_CC.read_text()
    rows = re.findall(r'\{"(\w+)",\s*\{([^}]*)\},\s*\{([^}]*)\},\s*\{([^}]*)\}\}',
                      text)
    assert len(rows) == len(mcw.TABLE)
    for name, a, b, kd in rows:
        values = tuple(tuple(float(x) for x in v.split(',')) for v in (a, b, kd))
        assert values == mcw.TABLE[name], name


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
    # With the camera's own model, other waters are well apart at the
    # default shot, as they are between renders.
    real, boxes = chart_image('3C')
    for water, at_least in (('7C', 10.0), ('5C', 5.0), ('1C', 5.0)):
        result = cv.compare(real, chart_image(water)[0], boxes)
        assert result['mean'] > at_least, water
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


def test_nearly_black_patches_are_dark(tmp_path):
    # A value of 1 in one channel is no colour: dark, not a 45 degree error
    real, boxes = chart_image('3C')
    sim = real.copy()
    for x0, y0, x1, y1 in boxes[:20]:
        sim[y0:y1, x0:x1] = (1, 0, 0)
    assert cv.compare(real, sim, boxes)['dark'] == 20


def test_too_many_dark_patches_rank_last(tmp_path):
    # A render with 20 black patches would score well on its 4 lit ones; it
    # ranks after a complete render, and the run passes.
    real, boxes = chart_image('3C')
    dark = real.copy()
    for x0, y0, x1, y1 in boxes[:20]:
        dark[y0:y1, x0:x1] = 0
    real_png, chart = save_chart(tmp_path, 'real', real, boxes)
    dark_png, _ = save_chart(tmp_path, 'dark', dark, boxes)
    other_png, _ = save_chart(tmp_path, '5C', chart_image('5C')[0], boxes)
    out = run_tool('--real', real_png, '--sim', f'dark={dark_png}',
                   '--sim', f'5C={other_png}', '--patches', chart,
                   '--json', str(tmp_path / 'result.json'))
    assert out.returncode == 0, out.stdout + out.stderr
    assert json.load(open(tmp_path / 'result.json'))['closest'] == ['5C', 'dark']


def test_a_lost_chart_is_refused(tmp_path):
    # 3C water, 3 m away: the patches all but one colour, every water alike
    real, boxes = chart_image('3C', rng_m=3.0, depth=5.0)
    assert cv.compare(real, real, boxes)['real_contrast'] < 3.0
    real_png, chart = save_chart(tmp_path, 'real', real, boxes)
    sim_png, _ = save_chart(tmp_path, 'sim', chart_image('3C')[0], boxes)
    out = run_tool('--real', real_png, '--sim', sim_png, '--patches', chart)
    # Not 1, a comparison that failed: the images cannot be compared
    assert out.returncode == 3
    assert 'lost in the water' in out.stderr


def test_a_repeated_name_is_refused(tmp_path):
    real, boxes = chart_image('3C')
    real_png, chart = save_chart(tmp_path, 'real', real, boxes)
    (tmp_path / 'a').mkdir()
    (tmp_path / 'b').mkdir()
    a_png, _ = save_chart(tmp_path / 'a', 'sim', real, boxes)
    b_png, _ = save_chart(tmp_path / 'b', 'sim', chart_image('7C')[0], boxes)
    out = run_tool('--real', real_png, '--sim', a_png, '--sim', b_png,
                   '--patches', chart)
    assert out.returncode == 2 and 'named sim' in out.stderr


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
    full = np.full((2, 2, 3), 65535, dtype=np.uint16)
    assert cv.srgb_to_linear(full).max() == pytest.approx(1.0)
    # An integer type whose scale cannot be known is refused
    with pytest.raises(ValueError, match='8 or 16 bit'):
        cv.srgb_to_linear(np.full((2, 2, 3), 255, dtype=np.int32))


def test_files_read_at_their_own_depth(tmp_path):
    from PIL import Image
    # 16 bit grey: not clipped at 255, the same as itself
    grey = (np.arange(64 * 64).reshape(64, 64) * 16).astype(np.uint16)
    Image.fromarray(grey).save(tmp_path / 'grey16.png')
    img = cv.load_image(tmp_path / 'grey16.png')
    assert img.dtype == np.uint16 and img.max() == grey.max()
    boxes = [[0, 0, 32, 32], [32, 32, 64, 64]]
    assert cv.compare(img, img, boxes)['luminance_ratio'] == pytest.approx(1.0)
    # Floating point in [0, 1]: not read as all black
    Image.fromarray(np.full((8, 8), 0.5, dtype=np.float32)).save(tmp_path / 'f.tiff')
    img = cv.load_image(tmp_path / 'f.tiff')
    assert cv.patch_means(img, [[0, 0, 8, 8]])[0] == pytest.approx(
        [0.214] * 3, abs=0.001)


def test_box_too_small_or_outside():
    img = np.zeros((10, 10, 3), dtype=np.uint8)
    with pytest.raises(ValueError, match='too small'):
        cv.patch_means(img, [[4, 2, 4, 8]])
    with pytest.raises(ValueError, match='outside'):
        cv.patch_means(img, [[5, 5, 20, 20]])


def test_luminance_ratio_is_reported():
    # An exposure difference: the same ratio on every patch. Reported, not
    # used to rank: another water can give one ratio everywhere too.
    real, boxes = chart_image('3C')
    exposure = cv.compare(real, chart_image('3C', gain=0.6)[0], boxes)
    assert exposure['luminance_ratio'] == pytest.approx(0.6, rel=0.05)
    assert exposure['luminance_spread'] < 0.05


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
    mcw.main(['--out', str(out), '--jerlov', '3C', '--width', '640', '--height', '360'])
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
    for bad in (['--jerlov', '4C'], ['--visibility', '0'],
                ['--visibility', '200'], ['--visibility', 'nan'],
                ['--jerlov', 'I', '--visibility', '50'], ['--range', '0'],
                ['--range', 'inf'], ['--hfov', '0'], ['--depth', '-1']):
        with pytest.raises(SystemExit):
            mcw.main(['--out', str(tmp_path / 'x.sdf'), *bad])


def render_chart(tmp_path, water):
    """Render the default chart world in a water; skip without a renderer."""
    import os
    world = tmp_path / f'chart_{water}.sdf'
    mcw.main(['--out', str(world), '--jerlov', water,
              '--width', '640', '--height', '360'])
    env = dict(os.environ, GZ_PARTITION=f'chart_test_{os.getpid()}_{water}')
    try:
        server = subprocess.Popen(['gz', 'sim', '-s', '-r', str(world)], env=env,
                                  stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        pytest.skip('no gz')
    try:
        png = tmp_path / f'chart_{water}.png'
        out = subprocess.run([sys.executable, str(TOOLS / 'save_frame.py'),
                              '--topic', '/chart/camera/underwater/image',
                              '--out', str(png), '--timeout', '40'],
                             env=env, capture_output=True, text=True)
    finally:
        server.terminate()
        server.wait(timeout=30)
    if out.returncode:
        pytest.skip('no frame rendered: no GPU, or the camera plugin is not '
                    'installed')
    return cv.load_image(png), cv.load_boxes(world.with_suffix('.json'))[1]


def test_rendered_chart_world(tmp_path):
    # The generated world, rendered: the boxes land on patches that differ,
    # and a chart drawn by the camera's model in 3C ranks the 3C render
    # first, ahead of 5C.
    pytest.importorskip('gz.transport')
    renders = {}
    for water in ('3C', '5C'):
        renders[water], boxes = render_chart(tmp_path, water)
        assert cv.contrast(cv.patch_means(renders[water], boxes)) > 10, water
    model, model_boxes = chart_image('3C', light=0.7)
    errors = {w: cv.compare(model, img, model_boxes, boxes)['mean']
              for w, img in renders.items()}
    assert errors['3C'] < 3.0, errors
    assert errors['5C'] > errors['3C'] + 4.0, errors
