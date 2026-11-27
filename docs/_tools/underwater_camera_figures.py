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
Draw the underwater camera plan's figures, and the results it must produce.

The schematics show how an underwater image forms and how each step is
tested. The rest is computed from the model the plan implements, so a built
step can be checked against it:

* the colour of a grey target against range, per channel, per water;
* the veil and a lit target against the camera's depth;
* a colour chart as the camera must see it at each range;
* a lamp's beam: its glow across the image, and the image itself;
* the Jerlov presets themselves: attenuation per channel, veil, visibility.

The waters are read from gz_water_optics' Jerlov table (src/Jerlov.cc) and
resolved as the optics system resolves them, so the figures are the expected
results for those presets, as far as the table is right: it is not yet
checked against the printed paper.

Needs numpy and matplotlib.

    python3 docs/_tools/underwater_camera_figures.py
"""

import os
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
# Text as text, not paths: smaller files that can be searched and diffed.
matplotlib.rcParams['svg.fonttype'] = 'none'
matplotlib.rcParams['svg.hashsalt'] = 'gz-maritime'
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyArrowPatch, Polygon, Rectangle  # noqa: E402

import numpy as np  # noqa: E402,I100

# Set UW_FIGURES_OUT to a directory to write the figures there instead, e.g.
# to compare them with the committed ones.
OUT = (Path(os.environ['UW_FIGURES_OUT']) if os.environ.get('UW_FIGURES_OUT')
       else Path(__file__).resolve().parent.parent / 'design' / 'images')
# Set UW_FIGURES_PREVIEW to a directory to also write PNG previews there.
PREVIEW = Path(os.environ['UW_FIGURES_PREVIEW']) if os.environ.get('UW_FIGURES_PREVIEW') else None

JERLOV_SOURCE = Path(__file__).resolve().parents[2] / 'gz_water_optics' / 'src' / 'Jerlov.cc'


def jerlov_table():
    """The Jerlov types from gz_water_optics' table: a, b, K_d per R G B."""
    text = JERLOV_SOURCE.read_text()
    num = r'([0-9.]+),\s*([0-9.]+),\s*([0-9.]+)'
    pat = re.compile(r'\{"(\w+)",\s*\{' + num + r'\},\s*\{' + num + r'\},\s*\{' + num + r'\}\}')
    table = {}
    for m in pat.finditer(text):
        v = [float(x) for x in m.groups()[1:]]
        a, b, kd = np.array(v[0:3]), np.array(v[3:6]), np.array(v[6:9])
        table[m.group(1)] = {'c': a + b, 'b': b, 'kd': kd}
    if len(table) != 10:
        raise SystemExit(f'expected 10 Jerlov types in {JERLOV_SOURCE}, found {len(table)}')
    return table


TABLE = jerlov_table()
# Three waters for the figures, as the optics system resolves them:
# attenuation = backscatter = c = a + b, veil = b / c, K_d as tabled.
WATERS = {
    'Jerlov IB, clear oceanic': TABLE['IB'],
    'Jerlov 3C, coastal': TABLE['3C'],
    'Jerlov 7C, turbid coastal': TABLE['7C'],
}
SUN = 1.0          # downwelling light at the surface, linear, per channel
DEPTH = 3.0        # the test scenes' camera depth, m
CH = ['red', 'green', 'blue']
CHC = ['#d1342f', '#2a9d55', '#2f6fd1']
INK, MUTE, GRID, BG = '#0b0b0b', '#52514e', '#e6e5e1', '#fcfcfb'
WATER = '#2f6f9f'


def veil(w, depth):
    """B-infinity at a depth: single-scatter albedo times the light there."""
    return w['b'] / w['c'] * SUN * np.exp(-w['kd'] * depth)


def seen(j, w, r, depth):
    """The revised model: object colour j (linear), range r, camera depth."""
    t = np.exp(-np.outer(np.atleast_1d(r), w['c']))
    return j * t + veil(w, depth) * (1 - t)


def to_srgb(x):
    x = np.clip(x, 0, 1)
    return np.where(x <= 0.0031308, 12.92 * x, 1.055 * x ** (1 / 2.4) - 0.055)


def style(ax):
    ax.set_facecolor(BG)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'):
        ax.spines[s].set_color('#c8c7c2')
    ax.tick_params(colors=MUTE)
    ax.grid(True, color=GRID, lw=0.8)


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.patch.set_facecolor(BG)
    # No date in the file, so the same figure is the same bytes.
    meta = {'Date': None} if name.endswith('.svg') else {'Software': None}
    fig.savefig(OUT / name, facecolor=BG, bbox_inches='tight', metadata=meta)
    if PREVIEW:
        PREVIEW.mkdir(parents=True, exist_ok=True)
        fig.savefig(PREVIEW / (Path(name).stem + '.png'), facecolor=BG, bbox_inches='tight', dpi=90)
    plt.close(fig)
    print(OUT / name)


def arrow(ax, a, b, color, lw=2, style='-|>', ls='-'):
    ax.add_patch(FancyArrowPatch(a, b, arrowstyle=style, mutation_scale=14,
                                 color=color, lw=lw, linestyle=ls))


def image_formation():
    """What reaches the camera, and from where."""
    fig, ax = plt.subplots(figsize=(10, 5.6))
    ax.set_xlim(0, 10)
    ax.set_ylim(-6, 1.6)
    ax.axis('off')
    ax.add_patch(Rectangle((0, -6), 10, 6, color='#dcebf5'))
    xs = np.linspace(0, 10, 200)
    ax.plot(xs, 0.12 * np.sin(xs * 2.2), color=WATER, lw=2)
    ax.text(0.1, 0.35, 'surface (the wave field)', color=WATER, fontsize=9)
    ax.text(8.6, 1.15, 'sun', color='#c98a00', fontsize=11, weight='bold')
    for x in (6.6, 7.6, 8.6):
        arrow(ax, (x + 0.4, 1.3), (x - 0.5, -3.9), '#e0a400', lw=1.6)
    ax.text(5.15, -1.2, 'sunlight fades with depth\n$E(z)=E_0\\,e^{-K_d z}$', fontsize=9, color='#8a6400')
    # camera and object
    ax.add_patch(Polygon([[0.6, -3.3], [1.4, -3.0], [1.4, -3.6]], color=INK))
    ax.text(0.2, -3.95, 'camera', fontsize=10, weight='bold')
    ax.add_patch(Rectangle((8.6, -4.6), 0.35, 1.6, color='#7a7a7a'))
    ax.text(8.15, -4.95, 'object $J$', fontsize=10, weight='bold')
    arrow(ax, (8.55, -3.8), (1.5, -3.3), '#1f77b4', lw=2.6)
    ax.text(3.3, -3.25, 'direct: $J\\,e^{-\\beta^D r}$', color='#1f77b4', fontsize=10)
    for x in (2.6, 4.2, 5.8, 7.2):
        y = -3.3 - (x - 1.5) * 0.07
        ax.plot([x], [y], 'o', color=MUTE, ms=3)
        arrow(ax, (x, y), (1.55, -3.3 + (x - 1.5) * 0.02 - 0.05), '#9a7fd1', lw=1.2, ls=(0, (3, 2)))
    ax.text(2.2, -2.55, 'backscatter: $B_\\infty(1-e^{-\\beta^B r})$', color='#7a5fc0', fontsize=10)
    # lamp and beam
    ax.add_patch(Polygon([[1.4, -4.4], [6.2, -5.9], [6.2, -4.2]], color='#ffe9a8', alpha=0.7))
    ax.add_patch(Rectangle((1.1, -4.55), 0.3, 0.3, color='#c98a00'))
    ax.text(0.15, -4.95, 'lamp', fontsize=10, weight='bold', color='#8a6400')
    ax.text(6.35, -5.35, 'the beam: lamp light\nscattered back to\nthe camera (§4.1)', fontsize=9,
            color='#8a6400')
    ax.text(0.1, 1.25, 'range $r$ along each ray, from depth; $\\beta^D \\neq \\beta^B$ (revised model)',
            fontsize=8.5, color=MUTE)
    ax.set_title('How an underwater image forms', loc='left', fontsize=13, weight='bold', color=INK)
    save(fig, 'uw-image-formation.svg')


def expected_range():
    """A 50 % grey target against range, per channel, for each water."""
    r = np.linspace(0, 20, 300)
    fig, axs = plt.subplots(1, 3, figsize=(12, 3.9), sharey=True)
    for ax, (name, w) in zip(axs, WATERS.items()):
        style(ax)
        y = to_srgb(seen(0.5 * SUN * np.exp(-w['kd'] * DEPTH), w, r, DEPTH))
        v = to_srgb(veil(w, DEPTH))
        for i in range(3):
            ax.plot(r, y[:, i], color=CHC[i], lw=2, label=CH[i])
            ax.axhline(v[i], color=CHC[i], lw=1, ls=(0, (4, 3)))
        ax.set_title(name, loc='left', color=INK, fontsize=11, weight='bold')
        ax.set_xlabel('range to the target (m)', color=MUTE)
    axs[0].set_ylabel('pixel value (sRGB, 0 to 1)', color=MUTE)
    axs[0].legend(frameon=False, fontsize=9)
    fig.suptitle('Expected: a 50 % grey target seen from 3 m deep. Solid: the target; dashed: the veil it fades into.',
                 x=0.01, ha='left', fontsize=11, color=INK)
    fig.text(0.01, -0.04, "Waters from gz_water_optics' Jerlov table (Solonenko and Mobley 2015). "
             'The reference camera must match these curves within one 8-bit step.', fontsize=8.5, color=MUTE)
    save(fig, 'uw-expected-range.svg')


def expected_depth():
    """The veil and a lit target against the camera's depth."""
    z = np.linspace(0, 40, 300)
    fig, axs = plt.subplots(1, 2, figsize=(10, 3.9))
    w = WATERS['Jerlov 3C, coastal']
    for ax, what in zip(axs, ('veil', 'target')):
        style(ax)
        for i in range(3):
            if what == 'veil':
                y = to_srgb(veil(w, z[:, None]))[:, i]
            else:
                j = 0.5 * SUN * np.exp(-w['kd'][i] * z)
                y = to_srgb(j * np.exp(-w['c'][i] * 2)
                            + veil(w, z[:, None])[:, i] * (1 - np.exp(-w['c'][i] * 2)))
            ax.plot(z, y, color=CHC[i], lw=2, label=CH[i])
        ax.set_xlabel('camera depth (m)', color=MUTE)
        ax.set_title('the veil (open water)' if what == 'veil' else 'a 50 % grey target 2 m away',
                     loc='left', fontsize=11, weight='bold', color=INK)
    axs[0].set_ylabel('pixel value (sRGB)', color=MUTE)
    axs[0].legend(frameon=False, fontsize=9)
    fig.suptitle('Expected: the scene darkens and shifts colour with depth (Jerlov 3C, no lamps)',
                 x=0.01, ha='left', fontsize=11, color=INK)
    save(fig, 'uw-expected-depth.svg')


def expected_chart():
    """A colour chart as the reference camera must see it, by range and water."""
    patches = np.array([[0.75, 0.75, 0.75], [0.20, 0.20, 0.20], [0.55, 0.12, 0.10],
                        [0.12, 0.45, 0.15], [0.10, 0.15, 0.55], [0.70, 0.60, 0.10]])
    ranges = [0.5, 1, 2, 4, 8, 16]
    p = 40
    rows = []
    for w in WATERS.values():
        row = []
        for r in ranges:
            cell = np.zeros((p * 2, p * 3, 3))
            for k, j in enumerate(patches):
                lit = j * SUN * np.exp(-w['kd'] * DEPTH)
                c = to_srgb(seen(lit, w, r, DEPTH))[0]
                cell[(k // 3) * p:(k // 3 + 1) * p, (k % 3) * p:(k % 3 + 1) * p] = c
            cell[:, -3:] = 1
            row.append(cell)
        rows.append(np.concatenate(row, axis=1))
        rows.append(np.ones((6, rows[-1].shape[1], 3)))
    img = np.concatenate(rows[:-1], axis=0)
    fig, ax = plt.subplots(figsize=(11, 4.6))
    ax.imshow(img, interpolation='nearest')
    ax.set_xticks([(i + 0.5) * p * 3 for i in range(len(ranges))], [f'{r:g} m' for r in ranges])
    ax.set_yticks([(i + 0.5) * (p * 2 + 6) for i in range(len(WATERS))], [k.split(',')[0] for k in WATERS])
    ax.tick_params(colors=MUTE, length=0)
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title("Expected: a six-patch chart seen from 3 m deep, at each range (the reference camera's target)",
                 loc='left', fontsize=11, weight='bold', color=INK)
    save(fig, 'uw-expected-chart.png')


def hg(cos_t, g=0.9):
    return (1 - g * g) / (4 * np.pi * (1 + g * g - 2 * g * cos_t) ** 1.5)


def beam_image(w, size=(120, 213), fov=1.2, lamp=(0.0, 0.35, -0.15), aim=(1, -0.12, 0.02),
               half=np.radians(35), far=12.0, steps=160, power=60.0):
    """Single scatter from one spot lamp, per pixel, ray-marched. Linear."""
    h, wd = size
    lamp = np.array(lamp)
    aim = np.array(aim) / np.linalg.norm(aim)
    fy = (wd / 2) / np.tan(fov / 2)
    v, u = np.mgrid[0:h, 0:wd]
    d = np.stack([np.full_like(u, fy, dtype=float), (wd / 2 - u), (h / 2 - v)], -1).astype(float)
    d /= np.linalg.norm(d, axis=-1, keepdims=True)
    ts = (np.arange(steps) + 0.5) * far / steps
    out = np.zeros((h, wd, 3))
    for t in ts:
        x = d * t
        to = x - lamp
        dl = np.linalg.norm(to, axis=-1)
        cos_cone = (to @ aim) / dl
        inside = (cos_cone > np.cos(half)).astype(float) * np.clip((cos_cone - np.cos(half)) / 0.05, 0, 1)
        cos_scatter = np.sum((to / dl[..., None]) * (-d), axis=-1)
        phase = hg(cos_scatter)
        e = power * inside[..., None] / (dl[..., None] ** 2) * np.exp(-w['c'] * dl[..., None])
        out += e * w['b'] * phase[..., None] * np.exp(-w['c'] * t) * (far / steps)
    return out


def expected_beam():
    """A lamp's beam in dark water: the image, and its glow across a row."""
    fig, axs = plt.subplots(2, 3, figsize=(12, 5.6), gridspec_kw={'height_ratios': [3, 2]})
    for k, (name, w) in enumerate(WATERS.items()):
        img = beam_image(w)
        axs[0, k].imshow(to_srgb(img / 0.25))
        axs[0, k].set_title(name, loc='left', fontsize=11, weight='bold', color=INK)
        axs[0, k].axis('off')
        row = img[img.shape[0] // 2]
        style(axs[1, k])
        for i in range(3):
            axs[1, k].plot(row[:, i] / max(img.max(), 1e-9), color=CHC[i], lw=1.8)
        axs[1, k].set_xlabel('image column (centre row)', color=MUTE, fontsize=9)
    axs[1, 0].set_ylabel('beam glow (relative)', color=MUTE)
    fig.suptitle('Expected: one lamp beside the camera, aimed ahead, at night, nothing in view but water (§4.1)',
                 x=0.01, ha='left', fontsize=11, color=INK)
    fig.text(0.01, -0.02, 'Single scatter, Henyey-Greenstein g = 0.9, 35° cone, 12 m. IB: a faint glow; '
             "7C: a bright haze on the lamp's side, close to the lens, that dies within metres.",
             fontsize=8.5, color=MUTE)
    save(fig, 'uw-expected-beam.png')


def test_scenes():
    """The rigs every step is tested in."""
    fig, axs = plt.subplots(1, 4, figsize=(13, 3.6))
    for ax in axs:
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 6)
        ax.axis('off')
        ax.add_patch(Rectangle((0, 0), 10, 5.2, color='#dcebf5'))
        ax.plot([0, 10], [5.2, 5.2], color=WATER, lw=2)
    # 1 range ladder
    a = axs[0]
    a.add_patch(Polygon([[0.3, 2.6], [1.0, 2.9], [1.0, 2.3]], color=INK))
    for k, x in enumerate([1.6, 2.3, 3.3, 4.8, 7.0, 9.6]):
        a.add_patch(Rectangle((x - 0.08, 1.9 + k * 0.02), 0.16, 1.4, color='#8a8a8a'))
    a.text(0.2, 0.5, 'Range ladder\nplates at 0.5 to 16 m', fontsize=9, color=INK)
    a.set_title('A', loc='left', weight='bold')
    # 2 depth elevator
    a = axs[1]
    for y in (4.4, 3.2, 2.0, 0.8):
        a.add_patch(Polygon([[3.0, y], [3.7, y + 0.3], [3.7, y - 0.3]], color=INK))
        a.add_patch(Rectangle((5.2, y - 0.5), 0.16, 1.0, color='#8a8a8a'))
    a.annotate('', (2.4, 0.6), (2.4, 4.8), arrowprops={'arrowstyle': '<->', 'color': MUTE})
    a.text(0.2, 0.15, 'Depth elevator: same view, 0 to 40 m', fontsize=9, color=INK)
    a.set_title('B', loc='left', weight='bold')
    # 3 CPU vs GPU
    a = axs[2]
    a.add_patch(Rectangle((0.6, 1.4), 3.6, 2.6, color='white', ec=MUTE))
    a.add_patch(Rectangle((5.6, 1.4), 3.6, 2.6, color='white', ec=MUTE))
    a.text(1.4, 2.6, 'CPU\nreference', fontsize=9)
    a.text(6.6, 2.6, 'GPU\npath', fontsize=9)
    a.text(4.45, 2.6, '−', fontsize=16)
    a.text(0.2, 0.3, 'Same scene, same frame:\ndifference within tolerance', fontsize=9, color=INK)
    a.set_title('C', loc='left', weight='bold')
    # 4 beam rig
    a = axs[3]
    a.add_patch(Rectangle((0, 0), 10, 5.2, color='#0d2a3f'))
    a.add_patch(Polygon([[0.3, 2.6], [1.0, 2.9], [1.0, 2.3]], color='white'))
    a.add_patch(Polygon([[1.1, 2.0], [9.8, 3.6], [9.8, 0.2]], color='#ffe9a8', alpha=0.45))
    a.add_patch(Rectangle((0.8, 1.85), 0.3, 0.3, color='#e0a400'))
    a.text(0.2, 4.4, 'Beam rig: night, black backdrop,\none lamp, three waters', fontsize=9, color='white')
    a.set_title('D', loc='left', weight='bold')
    fig.suptitle('Test scenes', x=0.01, ha='left', fontsize=13, weight='bold', color=INK)
    save(fig, 'uw-test-scenes.svg')


def presets():
    """Every Jerlov type: attenuation per channel, veil, visibility."""
    names = list(TABLE)
    x = np.arange(len(names))
    fig, axs = plt.subplots(1, 3, figsize=(13, 3.9))
    for ax in axs:
        style(ax)
        ax.set_xticks(x, names)
    for i in range(3):
        axs[0].bar(x + (i - 1) * 0.27, [TABLE[n]['c'][i] for n in names], 0.27, color=CHC[i], label=CH[i])
        axs[1].bar(x + (i - 1) * 0.27, [TABLE[n]['b'][i] / TABLE[n]['c'][i] for n in names], 0.27,
                   color=CHC[i])
    axs[0].set_yscale('log')
    axs[0].set_title('attenuation c = a + b (1/m, log)', loc='left', fontsize=11, weight='bold', color=INK)
    axs[0].legend(frameon=False, fontsize=9)
    axs[1].set_title('veil b / c (fraction of daylight)', loc='left', fontsize=11, weight='bold', color=INK)
    vis = [4.8 / TABLE[n]['c'][1] for n in names]
    axs[2].bar(x, vis, 0.6, color='#2a9d55')
    axs[2].set_yscale('log')
    for xi, v in zip(x, vis):
        axs[2].text(xi, v * 1.08, f'{v:.3g}', ha='center', fontsize=8, color=MUTE)
    axs[2].set_title('visibility 4.8 / c_green (m, log)', loc='left', fontsize=11, weight='bold', color=INK)
    fig.suptitle('The Jerlov presets of gz_water_optics (Solonenko and Mobley 2015; red 625, green 525, blue 450 nm)',
                 x=0.01, ha='left', fontsize=11, color=INK)
    save(fig, 'uw-jerlov-presets.svg')


if __name__ == '__main__':
    presets()
    image_formation()
    expected_range()
    expected_depth()
    expected_chart()
    expected_beam()
    test_scenes()
