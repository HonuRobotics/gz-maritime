#!/usr/bin/env python3
"""
Draw where one imaging ping's time goes, with a GPU and with llvmpipe.

The values are the measured stage times on the performance page
(docs/design/sonar-performance.md); the render is drawn at the middle of
its measured range. Writes ../images/sonar-ping-time.{svg,png}.
"""

import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'images', 'sonar-ping-time')

# (stage, ms, label, color); the first is on the rendering thread.
RUNS = {
    'With a GPU': [('Draw and copy back', 14.0, '13–15', '#3b6ea5'),
                   ('Images to hits', 18.0, '18', '#c9c9c9'),
                   ('Engine', 25.0, '25', '#9a9a9a'),
                   ('Levels and message', 11.0, '11', '#c9c9c9')],
    'llvmpipe (CPU only)': [('Draw and copy back', 20.5, '18–23', '#7f9fc4'),
                            ('Images to hits', 18.0, '18', '#c9c9c9'),
                            ('Engine', 25.0, '25', '#9a9a9a'),
                            ('Levels and message', 11.0, '11', '#c9c9c9')],
}

fig, ax = plt.subplots(figsize=(8.0, 2.6))
rows = list(RUNS)
for y, name in enumerate(reversed(rows)):
    x = 0.0
    for stage, ms, label, color in RUNS[name]:
        ax.barh(y, ms, left=x, height=0.55, color=color, edgecolor='white')
        ink = 'white' if color in ('#3b6ea5', '#9a9a9a') else 'black'
        ax.text(x + ms / 2, y, f'{label} ms', ha='center', va='center', fontsize=8, color=ink)
        x += ms
    ax.text(x + 1, y, f'{x:.0f} ms', va='center', fontsize=8)
ax.set_yticks(range(len(rows)), list(reversed(rows)))
ax.set_xlabel('milliseconds per ping')
ax.set_xlim(0, 85)
ax.set_ylim(-0.6, 2.0)
for x0, x1, text in ((0, 14, 'rendering thread:\nthe world waits'),
                     (14, 68, 'sonar worker thread, alongside the world:\nimages to hits, engine (all cores), levels')):
    ax.annotate('', xy=(x0, 1.45), xytext=(x1, 1.45),
                arrowprops=dict(arrowstyle='<->', lw=0.8))
    ax.text((x0 + x1) / 2, 1.55, text, ha='center', va='bottom', fontsize=8)
for side in ('top', 'right', 'left'):
    ax.spines[side].set_visible(False)
ax.tick_params(axis='y', length=0)
fig.tight_layout()
for ext in ('svg', 'png'):
    fig.savefig(f'{OUT}.{ext}', dpi=150)
