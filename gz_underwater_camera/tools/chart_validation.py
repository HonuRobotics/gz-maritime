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
Compare a simulated underwater image of a colour chart with a real one.

The metric is OceanSim's (arXiv:2503.01074): for each patch of the chart,
the angle between the real and the simulated mean colour, as RGB vectors in
linear light. It does not depend on brightness, only on hue and saturation,
so an exposure difference between the two cameras does not count; what the
water does to colour does.

    chart_validation.py --real real.png --sim sim.png --patches chart.json \
        [--report report.png] [--json result.json] [--threshold 10]

chart.json holds the patches' pixel boxes, the same in both images:

    {"patches": [{"name": "dark skin", "box": [x0, y0, x1, y1]}, ...]}

(make_chart_world.py writes it for a simulated scene; for real footage,
mark the boxes once by hand.) Each box is shrunk by a fifth on every side
before averaging, to stay clear of the patch's edges.

Exits 0 if the mean angular error is under the threshold (10 degrees by
default, the plan's target), 1 if not.
"""

import argparse
import json
import math
import sys

import numpy as np


def srgb_to_linear(v):
    """8-bit or [0, 1] sRGB to linear light."""
    v = np.asarray(v, dtype=float)
    if v.max() > 1.0:
        v = v / 255.0
    return np.where(v <= 0.04045, v / 12.92, ((v + 0.055) / 1.055) ** 2.4)


def angular_error(a, b):
    """Return the angle between two RGB colours, in degrees."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na == 0 or nb == 0:
        return float('nan')
    c = np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0)
    return math.degrees(math.acos(c))


def patch_means(image, boxes, shrink=0.2):
    """Return the mean linear colour of each box, shrunk by `shrink` per side."""
    lin = srgb_to_linear(image[..., :3])
    out = []
    for x0, y0, x1, y1 in boxes:
        dx, dy = (x1 - x0) * shrink, (y1 - y0) * shrink
        xa, xb = int(round(x0 + dx)), int(round(x1 - dx))
        ya, yb = int(round(y0 + dy)), int(round(y1 - dy))
        if xb <= xa or yb <= ya:
            raise ValueError(f'box {(x0, y0, x1, y1)} is too small')
        out.append(lin[ya:yb, xa:xb].reshape(-1, 3).mean(axis=0))
    return np.array(out)


def compare(real, sim, boxes):
    """Return the per-patch angular errors and their summary."""
    r = patch_means(real, boxes)
    s = patch_means(sim, boxes)
    errors = [angular_error(a, b) for a, b in zip(r, s)]
    valid = [e for e in errors if not math.isnan(e)]
    return {
        'errors': errors,
        'mean': float(np.mean(valid)) if valid else float('nan'),
        'median': float(np.median(valid)) if valid else float('nan'),
        'max': float(np.max(valid)) if valid else float('nan'),
        'real': r.tolist(),
        'sim': s.tolist(),
    }


def load_image(path):
    from PIL import Image
    return np.asarray(Image.open(path).convert('RGB'))


def report(path, real, sim, boxes, names, result, threshold):
    """Draw a figure: the two images with the boxes, and the error per patch."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig = plt.figure(figsize=(13, 7.5))
    for k, (img, title) in enumerate(((real, 'real'), (sim, 'simulated'))):
        ax = fig.add_subplot(2, 2, k + 1)
        ax.imshow(img)
        for x0, y0, x1, y1 in boxes:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec='white', lw=0.8))
        ax.set_title(title)
        ax.axis('off')
    ax = fig.add_subplot(2, 1, 2)
    x = np.arange(len(boxes))
    colours = ['#2a78d6' if e <= threshold else '#d1342f' for e in result['errors']]
    ax.bar(x, result['errors'], color=colours)
    ax.axhline(threshold, color='#52514e', ls=(0, (4, 3)), lw=1)
    ax.set_xticks(x, names, rotation=60, ha='right', fontsize=8)
    ax.set_ylabel('RGB angular error (degrees)')
    ax.set_title(f"mean {result['mean']:.1f}, median {result['median']:.1f}, "
                 f"max {result['max']:.1f} degrees (target: mean under {threshold:g})", loc='left')
    fig.tight_layout()
    fig.savefig(path, dpi=90)
    plt.close(fig)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    p.add_argument('--real', required=True, help='the real image')
    p.add_argument('--sim', required=True, help='the simulated image')
    p.add_argument('--patches', required=True, help='the patches JSON')
    p.add_argument('--report', help='write a report figure here')
    p.add_argument('--json', help='write the result here')
    p.add_argument('--threshold', type=float, default=10.0,
                   help='pass if the mean error is under this, degrees')
    a = p.parse_args(argv)
    spec = json.load(open(a.patches))
    boxes = [pt['box'] for pt in spec['patches']]
    names = [pt.get('name', str(i)) for i, pt in enumerate(spec['patches'])]
    real, sim = load_image(a.real), load_image(a.sim)
    if real.shape != sim.shape:
        sys.exit(f'the images differ in size: {real.shape} and {sim.shape}')
    result = compare(real, sim, boxes)
    for name, e in zip(names, result['errors']):
        print(f'{name:>16}: {e:5.1f} degrees')
    print(f"mean {result['mean']:.2f}, median {result['median']:.2f}, max {result['max']:.2f}")
    if a.report:
        report(a.report, real, sim, boxes, names, result, a.threshold)
    if a.json:
        json.dump(result, open(a.json, 'w'), indent=1)
    return 0 if result['mean'] < a.threshold else 1


if __name__ == '__main__':
    sys.exit(main())
