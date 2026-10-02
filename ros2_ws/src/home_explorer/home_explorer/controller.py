"""
Pure-Python explorer logic (no ROS imports), so it can be unit-tested and is
the exact same algorithm as demo/sim_demo.py.

Input : a LiDAR scan (ranges + beam angles in the robot frame, radians,
        0 = straight ahead, positive = left)
Output: (linear velocity m/s, angular velocity rad/s, state string)
"""

import math

import numpy as np


class ExplorerController:
    """
    Reactive gap-following explorer.

    1. If something is closer than `stop_dist` in front, rotate toward the
       side with more free space (and commit to that direction for a while
       so the robot doesn't dither).
    2. If something is brushing the robot's side, rotate away from it.
    3. Otherwise drive forward, steering toward the most open direction
       within +/- 90 degrees, slowing down as obstacles get closer.
       A slowly changing random "curiosity" heading keeps it from looping
       between the same two rooms forever.
    """

    def __init__(self, max_speed=0.35, max_turn=1.2, stop_dist=0.55, slow_dist=1.2,
                 side_dist=0.3, seed=None):
        self.max_speed = max_speed
        self.max_turn = max_turn
        self.stop_dist = stop_dist
        self.slow_dist = slow_dist
        self.side_dist = side_dist
        self.turn_dir = 0
        self.turn_ticks = 0
        self.rng = np.random.default_rng(seed)
        self.bias = 0.0
        self.bias_ticks = 0

    @staticmethod
    def sector_min(ranges, angles, lo, hi):
        m = (angles >= lo) & (angles <= hi)
        return float(ranges[m].min()) if m.any() else float("inf")

    @staticmethod
    def sector_mean(ranges, angles, lo, hi):
        m = (angles >= lo) & (angles <= hi)
        return float(ranges[m].mean()) if m.any() else 0.0

    def compute(self, ranges, angles, far=10.0):
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float)
        # wrap angles to [-pi, pi) - some lidars report 0..2pi
        angles = (angles + math.pi) % (2 * math.pi) - math.pi
        # "no return" (inf/nan) means open space as far as the sensor sees
        ranges = np.where(np.isfinite(ranges), ranges, far)

        front = self.sector_min(ranges, angles, -0.35, 0.35)
        left = self.sector_mean(ranges, angles, 0.35, 1.9)
        right = self.sector_mean(ranges, angles, -1.9, -0.35)

        # Committed turn-in-place (escaping a dead end)
        if self.turn_ticks > 0:
            self.turn_ticks -= 1
            return 0.0, self.turn_dir * self.max_turn, "turning"

        if front < self.stop_dist:
            self.turn_dir = 1 if left >= right else -1
            self.turn_ticks = 12
            return 0.0, self.turn_dir * self.max_turn, "obstacle"

        near_left = self.sector_min(ranges, angles, 0.35, 1.7)
        near_right = self.sector_min(ranges, angles, -1.7, -0.35)
        if min(near_left, near_right) < self.side_dist:
            w = -0.6 * self.max_turn if near_left < near_right else 0.6 * self.max_turn
            return 0.03, w, "too_close"

        if self.bias_ticks <= 0:
            self.bias = float(self.rng.uniform(-1.2, 1.2))
            self.bias_ticks = int(self.rng.integers(40, 120))   # 4-12 s at 10 Hz
        self.bias_ticks -= 1
        # the nudge fades out, so in open space it means "turn a bit, then go straight"
        self.bias *= 0.95

        fwd = (angles > -1.57) & (angles < 1.57)
        if not fwd.any():
            return 0.0, 0.0, "no_data"
        order = np.argsort(angles[fwd])
        a_f, r_f = angles[fwd][order], ranges[fwd][order]
        # smooth over ~0.25 rad regardless of how many beams the lidar has
        step = max(float(np.median(np.diff(a_f))) if len(a_f) > 1 else 0.05, 1e-3)
        k = max(3, int(0.25 / step) | 1)
        r_smooth = np.convolve(r_f, np.ones(k) / k, mode="same")
        score = np.minimum(r_smooth, 4.0) - 0.8 * np.abs(a_f - self.bias)
        target = float(a_f[int(np.argmax(score))])

        w = float(np.clip(1.5 * target, -self.max_turn, self.max_turn))
        speed_scale = float(np.clip((front - self.stop_dist) / (self.slow_dist - self.stop_dist),
                                    0.25, 1.0))
        v = self.max_speed * speed_scale * (1.0 - 0.5 * min(abs(target), 1.0))
        return float(v), w, "cruise"
