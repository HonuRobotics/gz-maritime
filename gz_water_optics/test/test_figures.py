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
"""The plan's figures, drawn from the Jerlov table, are the committed ones."""

import os
from pathlib import Path
import subprocess
import sys

import pytest

REPO = Path(__file__).resolve().parents[2]
SCRIPT = REPO / 'docs' / '_tools' / 'underwater_camera_figures.py'
IMAGES = REPO / 'docs' / 'design' / 'images'

# The matplotlib the figures were drawn with: another version draws the same
# figures with other bytes.
DRAWN_WITH = '3.10.7'


def test_figures_are_current(tmp_path):
    """Regenerate the figures: each is byte for byte the committed one."""
    matplotlib = pytest.importorskip('matplotlib')
    pytest.importorskip('numpy')
    if not matplotlib.__version__.startswith(DRAWN_WITH):
        pytest.skip(f'figures drawn with matplotlib {DRAWN_WITH}, '
                    f'not {matplotlib.__version__}')
    env = dict(os.environ, UW_FIGURES_OUT=str(tmp_path))
    run = subprocess.run([sys.executable, str(SCRIPT)], env=env,
                         capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr
    drawn = sorted(p.name for p in tmp_path.iterdir())
    assert len(drawn) == 7, drawn
    stale = [name for name in drawn
             if (tmp_path / name).read_bytes() != (IMAGES / name).read_bytes()]
    assert not stale, (f'{stale} differ from the committed figures: run '
                       f'{SCRIPT.relative_to(REPO)} and commit them')
