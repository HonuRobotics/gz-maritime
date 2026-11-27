#!/usr/bin/env python3
"""
Draw the shipwreck scene's real time factor for each set of sonars, with a GPU and with llvmpipe.

The values are the measurements on the performance page
(docs/design/sonar-performance.md). Writes ../images/sonar-rtf.{svg,png}.
"""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'images', 'sonar-rtf')
CASES = [('No sonar', 0.992, 0.993),
         ('All three, no subscriber', 0.993, 0.993),
         ('ROV profiler', 0.916, 0.932),
         ('USV sidescan', 0.885, 0.900),
         ('ROV imaging', 0.780, 0.738),
         ('All three', 0.709, 0.640)]

fig, ax = plt.subplots(figsize=(7.5, 3.9))
h = 0.36
for i, (name, gpu, cpu) in enumerate(reversed(CASES)):
    for dy, v, color, label in ((h / 2, gpu, '#3b6ea5', 'With a GPU'), (-h / 2, cpu, '#a8a8a8', 'llvmpipe (CPU only)')):
        ax.barh(i + dy, v, height=h, color=color, label=label if i == 0 else None)
        ax.text(v + 0.01, i + dy, f'{v:.2f}', va='center', fontsize=8)
ax.set_yticks(range(len(CASES)), [c[0] for c in reversed(CASES)])
ax.set_xlim(0, 1.12)
ax.set_xlabel('real time factor (1.0 is real time)')
ax.legend(loc='lower right', fontsize=8, frameon=False)
for side in ('top', 'right'):
    ax.spines[side].set_visible(False)
fig.tight_layout()
for ext in ('svg', 'png'):
    fig.savefig(f'{OUT}.{ext}', dpi=150)
