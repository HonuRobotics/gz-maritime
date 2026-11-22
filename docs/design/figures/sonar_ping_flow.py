#!/usr/bin/env python3
"""
Draw one sonar ping as a swimlane chart: a lane per thread, the GPU step shaded.

Writes ../images/sonar-ping-flow.{svg,png}.
"""

import os

import matplotlib
matplotlib.use('Agg')
from matplotlib.patches import FancyBboxPatch  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'images', 'sonar-ping-flow')
CPU, GPU = '#eeeeee', '#cfdcec'

LANES = [('Simulation thread', 2.0), ('Rendering thread', 1.0), ('Sonar worker thread', 0.0)]
# (lane y, x, text, color)
BOXES = {
    'step': (2.0, 0.0, 'gz-sim\nsteps the world', CPU),
    'scene': (1.0, 1.45, 'Sensors system\nupdates the scene', CPU),
    'trigger': (1.0, 2.9, 'sonar plugin\nping due: render', CPU),
    'draw': (1.0, 4.35, 'gz-rendering\ndraws depth and\nsegmentation,\ncopies to CPU', GPU),
    'hits': (0.0, 5.8, 'gz_sonar\nimages to hits\n1 core', CPU),
    'engine': (0.0, 7.25, 'gz_sonar engine\nbeams, samples\nall cores', CPU),
    'out': (0.0, 8.7, 'gz_sonar\nlevels, message\n1 core', CPU),
}
W, H = 1.2, 0.7

fig, ax = plt.subplots(figsize=(9.5, 3.6))
for name, y in LANES:
    ax.axhspan(y - 0.45, y + 0.45, color='#fafafa', zorder=0)
    ax.text(-0.75, y, name, ha='right', va='center', fontsize=9)
for y, x, text, color in BOXES.values():
    ax.add_patch(FancyBboxPatch((x - W / 2, y - H / 2), W, H, boxstyle='round,pad=0.02',
                                fc=color, ec='#555555', lw=0.8, zorder=2))
    ax.text(x, y, text, ha='center', va='center', fontsize=7.5, zorder=3)


def arrow(a, b, text='', style='-', dy=0.0):
    (ya, xa), (yb, xb) = BOXES[a][:2], BOXES[b][:2]
    start = (xa + W / 2, ya) if ya == yb else (xa + W / 2, ya - (H / 2 if yb < ya else -H / 2))
    end = (xb - W / 2, yb) if ya == yb else (xb - W / 2, yb)
    ax.annotate('', xy=end, xytext=start, zorder=1,
                arrowprops=dict(arrowstyle='->', lw=0.9, ls=style, color='#333333'))
    if text:
        ax.text((start[0] + end[0]) / 2, (start[1] + end[1]) / 2 + dy, text,
                ha='center', va='bottom', fontsize=7.5, color='#333333')


arrow('step', 'scene', 'sensor due', dy=0.05)
arrow('scene', 'trigger')
arrow('trigger', 'draw')
arrow('draw', 'hits', 'up to 2 wait', dy=0.05)
arrow('hits', 'engine')
arrow('engine', 'out')
ax.annotate('', xy=(BOXES['step'][1] + W / 2 + 0.05, 2.0 - 0.08),
            xytext=(BOXES['draw'][1], 1.0 + H / 2), zorder=1,
            arrowprops=dict(arrowstyle='->', lw=0.9, ls='--', color='#333333',
                            connectionstyle='angle,angleA=90,angleB=0,rad=0'))
ax.text(2.2, 1.97, 'rendering done: the world goes on', fontsize=7.5, color='#333333')
ax.add_patch(FancyBboxPatch((7.0, 2.35), 0.25, 0.18, boxstyle='square,pad=0', fc=GPU, ec='#555555', lw=0.8))
ax.text(7.32, 2.44, 'GPU (CPU with llvmpipe)', va='center', fontsize=7.5)
ax.add_patch(FancyBboxPatch((7.0, 2.08), 0.25, 0.18, boxstyle='square,pad=0', fc=CPU, ec='#555555', lw=0.8))
ax.text(7.32, 2.17, 'CPU', va='center', fontsize=7.5)
ax.set_xlim(-2.4, 9.4)
ax.set_ylim(-0.55, 2.65)
ax.axis('off')
fig.tight_layout()
for ext in ('svg', 'png'):
    fig.savefig(f'{OUT}.{ext}', dpi=150)
