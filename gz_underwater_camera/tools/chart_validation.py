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
Compare simulated underwater images of a colour chart with a real one.

For each patch of the chart, two numbers, in linear light:

* the angle between the real and the simulated mean colour, as RGB vectors
  (OceanSim's metric, arXiv:2503.01074): hue and saturation, not brightness;
* the ratio of their luminance, simulated over real: brightness. It is
  reported, not used: an exposure difference gives the same ratio on every
  patch, but so can another water, so it does not tell them apart.

    chart_validation.py --real dive.png --real-patches dive.json \
        --sim 3C=sim_3c.png --sim 5C=sim_5c.png --sim-patches chart.json \
        [--report report.png] [--json result.json] [--threshold 10]

The boxes of the patches are given for each image, since a photo never
lines up pixel for pixel with a render: --real-patches for the photo (mark
them once by hand), --sim-patches for the renders (make_chart_world.py
writes them). --patches gives the same boxes for both. A boxes file holds

    {"patches": [{"name": "dark skin", "box": [x0, y0, x1, y1]}, ...]}

in the same order for both. Each box is shrunk by a fifth on every side
before averaging, to stay clear of the patch's edges.

With one simulated image or several, one per water (name=image), it prints
each water's errors, closest first: the nearest water types. There is no
pass or fail, unless --threshold is given: then it exits 1 if the closest
water's mean error is not under it. A patch too dark to have a colour in
either image (linear luminance under 0.001) is left out and counted; a
water with more than --max-dark of them ranks after every other, and if it
is the closest, it exits 1.

An image whose patches differ from one another by less than --min-contrast
degrees, at most, is refused (exit 3, apart from a failed comparison's 1
and a bad command line's 2): the chart is lost in the water, and every
water would look alike. Come closer, or into clearer water.

Images are read at their own depth: 8 or 16 bit, or floating point in
[0, 1]. The photo needs the white balance the simulation has (none, by
default): a 10 % white balance error moves the ranking.
"""

import argparse
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np

# Rec. 709 luminance weights, for linear RGB.
LUMINANCE = np.array([0.2126, 0.7152, 0.0722])

# A patch darker than this, in linear luminance, has no colour to compare.
DARK = 1e-3

# The full scale of the integer types an image file holds.
SCALES = {np.dtype(np.uint8): 255.0, np.dtype(np.uint16): 65535.0}


def srgb_to_linear(image):
    """
    Return an sRGB image in linear light.

    8 and 16 bit images are scaled by their full scale, floating point
    images are taken as [0, 1]; any other type is refused, as its scale
    cannot be known.
    """
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.integer):
        if image.dtype not in SCALES:
            raise ValueError(f'{image.dtype} images: give 8 or 16 bit, or '
                             'floating point in [0, 1]')
        v = image.astype(float) / SCALES[image.dtype]
    else:
        v = image.astype(float)
    return np.where(v <= 0.04045, v / 12.92, ((v + 0.055) / 1.055) ** 2.4)


def angular_error(a, b):
    """Return the angle between two RGB colours, in degrees; NaN if black."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0 or nb == 0:
        return float('nan')
    c = np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)
    return math.degrees(math.acos(c))


def patch_means(image, boxes, shrink=0.2):
    """Return the mean linear colour of each box, shrunk by `shrink` per side."""
    image = np.asarray(image)
    if image.ndim == 2:
        image = np.repeat(image[..., None], 3, axis=2)
    lin = srgb_to_linear(image[..., :3])
    out = []
    for x0, y0, x1, y1 in boxes:
        dx, dy = (x1 - x0) * shrink, (y1 - y0) * shrink
        xa, xb = int(round(x0 + dx)), int(round(x1 - dx))
        ya, yb = int(round(y0 + dy)), int(round(y1 - dy))
        if xb <= xa or yb <= ya:
            raise ValueError(f'box {(x0, y0, x1, y1)} is too small')
        if xa < 0 or ya < 0 or xb > lin.shape[1] or yb > lin.shape[0]:
            raise ValueError(f'box {(x0, y0, x1, y1)} is outside the image')
        out.append(lin[ya:yb, xa:xb].reshape(-1, 3).mean(axis=0))
    return np.array(out)


def compare(real, sim, real_boxes, sim_boxes=None):
    """Return the per-patch angular errors and luminance ratios, summarised."""
    if sim_boxes is None:
        sim_boxes = real_boxes
    if len(real_boxes) != len(sim_boxes):
        raise ValueError(f'{len(real_boxes)} real patches, {len(sim_boxes)} '
                         'simulated: they must be the same, in order')
    r = patch_means(real, real_boxes)
    s = patch_means(sim, sim_boxes)
    errors = [angular_error(a, b) if min(a @ LUMINANCE, b @ LUMINANCE) >= DARK
              else float('nan') for a, b in zip(r, s)]
    ratios = [float(b @ LUMINANCE / (a @ LUMINANCE)) if a @ LUMINANCE > 0
              else float('nan') for a, b in zip(r, s)]
    valid = [e for e in errors if not math.isnan(e)]
    good = [q for q in ratios if not math.isnan(q) and q > 0]
    nan = float('nan')
    return {
        'errors': errors,
        'dark': len(errors) - len(valid),
        'mean': float(np.mean(valid)) if valid else nan,
        'median': float(np.median(valid)) if valid else nan,
        'max': float(np.max(valid)) if valid else nan,
        'luminance_ratios': ratios,
        # The typical ratio, and how much it varies between patches (the
        # standard deviation of its logarithm): 0 for an exposure difference
        'luminance_ratio': float(np.median(good)) if good else nan,
        'luminance_spread': float(np.std(np.log(good))) if good else nan,
        'real': r.tolist(),
        'sim': s.tolist(),
        'real_contrast': contrast(r),
        'sim_contrast': contrast(s),
    }


def contrast(means):
    """Return the largest angle between two patches of one image, degrees."""
    lit = [m for m in means if m @ LUMINANCE >= DARK]
    angles = [angular_error(a, b) for a, b in itertools.combinations(lit, 2)]
    return max(angles) if angles else 0.0


def json_safe(value):
    """Return a value with every NaN as None, so it is valid JSON."""
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and math.isnan(value):
        return None
    return value


def load_image(path):
    """Return an image file at its own depth: 8 or 16 bit, or float."""
    from PIL import Image
    image = Image.open(path)
    if image.mode in ('I', 'I;16', 'I;16B', 'I;16L', 'I;16N'):
        # 16 bit grey (Pillow holds it as 32 bit integers)
        return np.asarray(image).astype(np.int64).clip(0, 65535).astype(
            np.uint16)
    if image.mode == 'F':
        return np.asarray(image, dtype=np.float32)
    return np.asarray(image.convert('RGB'))


def load_boxes(path):
    """Return the names and boxes of a patches file."""
    spec = json.load(open(path))
    names = [pt.get('name', str(i)) for i, pt in enumerate(spec['patches'])]
    return names, [pt['box'] for pt in spec['patches']]


def report(path, real, real_boxes, sims, sim_boxes, names, results, order,
           threshold):
    """Draw a figure: the real and the closest simulated image, and the errors."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    best = order[0]
    fig = plt.figure(figsize=(13, 7.5))
    for k, (img, boxes, title) in enumerate(
            ((real, real_boxes, 'real'), (sims[best], sim_boxes, f'simulated, {best}'))):
        ax = fig.add_subplot(2, 2, k + 1)
        ax.imshow(img)
        for x0, y0, x1, y1 in boxes:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='white', lw=0.8))
        ax.set_title(title)
        ax.axis('off')
    ax = fig.add_subplot(2, 1, 2)
    x = np.arange(len(names))
    width = 0.8 / len(results)
    for k, water in enumerate(order):
        r = results[water]
        ax.bar(x + k * width, [0 if math.isnan(e) else e for e in r['errors']], width,
               label=f"{water}: mean {r['mean']:.1f} degrees")
    if threshold is not None:
        ax.axhline(threshold, color='#52514e', ls=(0, (4, 3)), lw=1)
    ax.set_xticks(x + 0.4 - width / 2, names, rotation=60, ha='right', fontsize=8)
    ax.set_ylabel('RGB angular error (degrees)')
    ax.legend(loc='upper left', fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def parse_sim(text):
    """Return (water, path) of a --sim argument, water=path or a path."""
    if '=' in text:
        water, path = text.split('=', 1)
        return water, path
    return Path(text).stem, text


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--real', required=True, help='the real image')
    p.add_argument('--sim', required=True, action='append',
                   help='a simulated image, as water=path or path; repeat it '
                        'for each water to compare')
    p.add_argument('--patches', help='the boxes, the same in every image')
    p.add_argument('--real-patches', help='the boxes in the real image')
    p.add_argument('--sim-patches', help='the boxes in the simulated images')
    p.add_argument('--report', help='write a report figure here')
    p.add_argument('--json', help='write the results here')
    p.add_argument('--threshold', type=float,
                   help='exit 1 if the closest mean error is not under this, '
                        'degrees; by default there is no pass or fail')
    p.add_argument('--max-dark', type=int, default=4,
                   help='patches too dark to compare, at most (default 4)')
    p.add_argument('--min-contrast', type=float, default=3.0,
                   help='refuse an image whose patches differ by less than '
                        'this, at most, degrees (default 3)')
    a = p.parse_args(argv)
    real_path = a.real_patches or a.patches
    sim_path = a.sim_patches or a.patches
    if not real_path or not sim_path:
        p.error('give --patches, or --real-patches and --sim-patches')
    names, real_boxes = load_boxes(real_path)
    _, sim_boxes = load_boxes(sim_path)

    real = load_image(a.real)
    sims, results = {}, {}
    for text in a.sim:
        water, path = parse_sim(text)
        if water in sims:
            p.error(f'two simulated images named {water}: name them, '
                    'as water=path')
        sims[water] = load_image(path)
        results[water] = compare(real, sims[water], real_boxes, sim_boxes)

    # The patches of every image must differ, or every water looks alike
    flat = []
    first = next(iter(results.values()))
    if first['real_contrast'] < a.min_contrast:
        flat.append(f"the real image's ({first['real_contrast']:.1f})")
    flat += [f"{w}'s ({r['sim_contrast']:.1f})" for w, r in results.items()
             if r['sim_contrast'] < a.min_contrast]
    if flat:
        print(f'the patches differ by less than {a.min_contrast} degrees in '
              f'{", ".join(flat)}: the chart is lost in the water; come '
              'closer, or into clearer water', file=sys.stderr)
        return 3

    # Those with too many dark patches last: their few patches say little
    order = sorted(results, key=lambda w: (results[w]['dark'] > a.max_dark,
                                           math.isnan(results[w]['mean']),
                                           results[w]['mean']))
    for water in order:
        r = results[water]
        print(f"{water}: mean {r['mean']:.2f}, median {r['median']:.2f}, "
              f"max {r['max']:.2f} degrees; luminance {r['luminance_ratio']:.2f} "
              f"times the real, spread {r['luminance_spread']:.2f}; "
              f"{r['dark']} patches too dark")
    best = results[order[0]]
    print(f'closest: {", ".join(order)}')
    for name, e in zip(names, best['errors']):
        print(f'{name:>16}: {e:5.1f} degrees')

    if a.report:
        report(a.report, real, real_boxes, sims, sim_boxes, names, results,
               order, a.threshold)
    if a.json:
        json.dump(json_safe({'closest': order, 'waters': results}),
                  open(a.json, 'w'), indent=1, allow_nan=False)

    if best['dark'] > a.max_dark:
        print(f"{best['dark']} patches too dark to compare, more than "
              f'{a.max_dark}', file=sys.stderr)
        return 1
    if a.threshold is not None and not best['mean'] < a.threshold:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
