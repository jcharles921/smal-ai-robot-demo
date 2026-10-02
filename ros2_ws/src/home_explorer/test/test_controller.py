"""Unit tests for the explorer logic - run with:  python3 -m pytest test/"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from home_explorer.controller import ExplorerController  # noqa: E402

ANGLES = np.linspace(-math.pi, math.pi, 360, endpoint=False)


def test_open_space_drives_forward():
    c = ExplorerController(seed=0)
    v, w, state = c.compute(np.full(360, 5.0), ANGLES)
    assert state == "cruise" and v > 0


def test_wall_ahead_turns_in_place():
    r = np.full(360, 5.0)
    r[np.abs(ANGLES) < 0.3] = 0.3
    v, w, state = ExplorerController(seed=0).compute(r, ANGLES)
    assert state == "obstacle" and v == 0.0 and w != 0.0


def test_turns_toward_open_side():
    r = np.full(360, 5.0)
    r[np.abs(ANGLES) < 0.3] = 0.3
    r[ANGLES < -0.3] = 0.8            # right side cluttered
    _, w, _ = ExplorerController(seed=0).compute(r, ANGLES)
    assert w > 0                      # positive = turn left


def test_infinite_ranges_and_0_to_2pi_angles():
    a = np.linspace(0, 2 * math.pi, 360, endpoint=False)
    v, _, state = ExplorerController(seed=0).compute(np.full(360, np.inf), a)
    assert state == "cruise" and v > 0
