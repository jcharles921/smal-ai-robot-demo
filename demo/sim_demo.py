#!/usr/bin/env python3
"""
Home Robot Starter - zero-install demo
======================================

A tiny 2D version of what linorobot2 + Gazebo + SLAM Toolbox do, in one file:

  * a differential-drive robot (same kinematics as linorobot2's "2wd" base)
  * a simulated 360-degree LiDAR (like the /scan topic)
  * a reactive "explorer" controller (the same logic as the ROS 2 node in
    ros2_ws/src/home_explorer) that turns the scan into velocity commands
    (like the /cmd_vel topic)
  * an occupancy-grid map built live from the scans (what SLAM Toolbox
    publishes on /map - here with perfect odometry, so it's "mapping with
    known poses" rather than full SLAM)

Only needs numpy + matplotlib:
    pip install numpy matplotlib
    python sim_demo.py

Controls (click the window first):
    m        toggle AUTO / MANUAL driving
    arrows   drive in MANUAL mode (up/down = speed, left/right = turn)
    space    stop (manual mode)
    r        reset robot and map
    p        save the current map to map.png
    q        quit
"""

import argparse
import math
import sys

import numpy as np

# ---------------------------------------------------------------------------
# World: a small apartment, described as wall segments (metres)
# ---------------------------------------------------------------------------

WORLD_W, WORLD_H = 10.0, 8.0


def box(x0, y0, x1, y1):
    """Four segments of an axis-aligned rectangle (a piece of furniture)."""
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def build_world():
    segs = []
    # outer walls
    segs += box(0, 0, WORLD_W, WORLD_H)
    # interior walls with door gaps
    segs += [(4.0, 0.0, 4.0, 2.6), (4.0, 3.6, 4.0, 8.0)]      # living | bedroom wall
    segs += [(4.0, 4.5, 6.0, 4.5), (7.0, 4.5, 10.0, 4.5)]     # bedroom | kitchen wall
    segs += [(0.0, 5.0, 1.6, 5.0), (2.6, 5.0, 4.0, 5.0)]      # living | hallway wall
    # furniture
    segs += box(1.0, 1.0, 2.6, 1.8)    # sofa
    segs += box(1.6, 3.0, 2.4, 3.6)    # coffee table
    segs += box(7.4, 0.6, 9.4, 2.2)    # bed
    segs += box(5.0, 6.0, 6.2, 7.0)    # kitchen table
    segs += box(8.8, 5.0, 9.6, 7.6)    # kitchen counter
    segs += box(0.6, 6.4, 1.2, 7.4)    # wardrobe
    return np.array(segs, dtype=float)


# ---------------------------------------------------------------------------
# Sensors and physics
# ---------------------------------------------------------------------------

class Lidar:
    """Raycasting 2D LiDAR. Returns ranges like sensor_msgs/LaserScan."""

    def __init__(self, n_beams=180, max_range=6.0, noise_std=0.01, rng=None):
        self.n = n_beams
        self.max_range = max_range
        self.noise = noise_std
        self.rel_angles = np.linspace(-math.pi, math.pi, n_beams, endpoint=False)
        self.rng = rng or np.random.default_rng()

    def scan(self, x, y, th, segs):
        ang = th + self.rel_angles                      # (N,)
        dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
        x1, y1, x2, y2 = (segs[:, i][None, :] for i in range(4))
        ex, ey = x2 - x1, y2 - y1                       # (1,M)
        denom = dx * ey - dy * ex                       # (N,M)
        with np.errstate(divide="ignore", invalid="ignore"):
            t = ((x1 - x) * ey - (y1 - y) * ex) / denom   # distance along ray
            u = ((x1 - x) * dy - (y1 - y) * dx) / denom   # position along segment
        hit = (np.abs(denom) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        t = np.where(hit, t, np.inf)
        r = t.min(axis=1)
        r = r + self.rng.normal(0, self.noise, r.shape)
        return np.clip(r, 0.05, self.max_range), self.rel_angles


def point_segment_dist(px, py, segs):
    x1, y1, x2, y2 = segs.T
    ex, ey = x2 - x1, y2 - y1
    L2 = ex * ex + ey * ey
    t = np.clip(((px - x1) * ex + (py - y1) * ey) / np.maximum(L2, 1e-12), 0, 1)
    cx, cy = x1 + t * ex, y1 + t * ey
    return np.hypot(px - cx, py - cy)


class DiffDriveRobot:
    """Unicycle model of a 2-wheel differential drive base."""

    RADIUS = 0.18  # m

    def __init__(self, x=2.0, y=2.4, th=0.0):
        self.reset(x, y, th)

    def reset(self, x=2.0, y=2.4, th=0.0):
        self.x, self.y, self.th = x, y, th
        self.v = self.w = 0.0
        self.odom_dist = 0.0
        self.bumps = 0

    def step(self, v, w, dt, segs):
        self.v, self.w = v, w
        nth = self.th + w * dt
        nx = self.x + v * math.cos(nth) * dt
        ny = self.y + v * math.sin(nth) * dt
        if point_segment_dist(nx, ny, segs).min() > self.RADIUS:
            self.odom_dist += abs(v) * dt
            self.x, self.y = nx, ny
        else:
            self.bumps += 1           # bumper hit: rotate in place only
        self.th = (nth + math.pi) % (2 * math.pi) - math.pi


# ---------------------------------------------------------------------------
# Controller - identical logic to home_explorer/explorer_node.py
# ---------------------------------------------------------------------------

class ExplorerController:
    """
    Reactive gap-following explorer.

    1. If something is closer than `stop_dist` in front, rotate toward the
       side with more free space (and commit to that direction for a while
       so the robot doesn't dither).
    2. Otherwise drive forward, steering toward the most open direction
       within +/- 90 degrees, slowing down as obstacles get closer.
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
        # "Curiosity": a preferred heading offset that changes every few
        # seconds, so the robot doesn't loop forever between the same rooms.
        self.rng = np.random.default_rng(seed)
        self.bias = 0.0
        self.bias_ticks = 0

    @staticmethod
    def sector_min(ranges, angles, lo, hi):
        m = (angles >= lo) & (angles <= hi)
        return float(ranges[m].min()) if m.any() else float("inf")

    def compute(self, ranges, angles):
        ranges = np.asarray(ranges, dtype=float)
        angles = np.asarray(angles, dtype=float)
        ranges = np.where(np.isfinite(ranges), ranges, 10.0)

        front = self.sector_min(ranges, angles, -0.35, 0.35)
        left = np.mean(ranges[(angles > 0.35) & (angles < 1.9)])
        right = np.mean(ranges[(angles < -0.35) & (angles > -1.9)])

        # Committed turn-in-place (escaping a dead end)
        if self.turn_ticks > 0:
            self.turn_ticks -= 1
            return 0.0, self.turn_dir * self.max_turn, "turning"

        if front < self.stop_dist:
            self.turn_dir = 1 if left >= right else -1
            self.turn_ticks = 12
            return 0.0, self.turn_dir * self.max_turn, "obstacle"

        # Something brushing the side (e.g. a door frame): rotate away from it
        near_left = self.sector_min(ranges, angles, 0.35, 1.7)
        near_right = self.sector_min(ranges, angles, -1.7, -0.35)
        if min(near_left, near_right) < self.side_dist:
            w = -0.6 * self.max_turn if near_left < near_right else 0.6 * self.max_turn
            return 0.03, w, "too close"

        # Gap following: smooth the scan, pick the most open heading ahead
        if self.bias_ticks <= 0:
            self.bias = float(self.rng.uniform(-1.2, 1.2))
            self.bias_ticks = int(self.rng.integers(40, 120))   # 4-12 s at 10 Hz
        self.bias_ticks -= 1
        # the nudge fades out, so in open space it means "turn a bit, then go straight"
        self.bias *= 0.95

        fwd = (angles > -1.57) & (angles < 1.57)
        a_f, r_f = angles[fwd], ranges[fwd]
        k = 7
        r_smooth = np.convolve(r_f, np.ones(k) / k, mode="same")
        # open space is good; drifting far from the current "curiosity" heading is bad
        score = np.minimum(r_smooth, 4.0) - 0.8 * np.abs(a_f - self.bias)
        target = float(a_f[int(np.argmax(score))])

        w = float(np.clip(1.5 * target, -self.max_turn, self.max_turn))
        speed_scale = np.clip((front - self.stop_dist) / (self.slow_dist - self.stop_dist), 0.25, 1.0)
        v = self.max_speed * speed_scale * (1.0 - 0.5 * min(abs(target), 1.0))
        return float(v), w, "cruise"


# ---------------------------------------------------------------------------
# Mapping - log-odds occupancy grid (what SLAM Toolbox publishes on /map)
# ---------------------------------------------------------------------------

class OccupancyGrid:
    L_OCC, L_FREE, L_MIN, L_MAX = 0.85, -0.6, -4.0, 4.0

    def __init__(self, width=WORLD_W, height=WORLD_H, res=0.05):
        self.res = res
        self.nx, self.ny = int(width / res), int(height / res)
        self.logodds = np.zeros((self.ny, self.nx), dtype=np.float32)

    def reset(self):
        self.logodds[:] = 0

    def update(self, x, y, th, ranges, angles, max_range):
        ang = th + angles
        # free space: sample points along every beam (vectorised "ray tracing")
        steps = np.arange(0, max_range, self.res * 0.8)
        d = steps[None, :]
        valid = d < (ranges[:, None] - self.res)
        fx = (x + d * np.cos(ang)[:, None])[valid]
        fy = (y + d * np.sin(ang)[:, None])[valid]
        fi, fj = self._idx(fx, fy)
        free = np.zeros_like(self.logodds, dtype=bool)
        free[fj, fi] = True
        self.logodds[free] += self.L_FREE
        # occupied: beam endpoints that actually hit something
        hit = ranges < max_range - 0.05
        hx = x + ranges[hit] * np.cos(ang[hit])
        hy = y + ranges[hit] * np.sin(ang[hit])
        hi, hj = self._idx(hx, hy)
        occ = np.zeros_like(free)
        occ[hj, hi] = True
        self.logodds[occ] += self.L_OCC - self.L_FREE * free[occ]
        np.clip(self.logodds, self.L_MIN, self.L_MAX, out=self.logodds)

    def _idx(self, x, y):
        i = np.clip((x / self.res).astype(int), 0, self.nx - 1)
        j = np.clip((y / self.res).astype(int), 0, self.ny - 1)
        return i, j

    def image(self):
        """0 = free (white), 1 = occupied (black), 0.5 = unknown (grey)."""
        p_occ = 1 - 1 / (1 + np.exp(self.logodds))
        img = 1 - p_occ
        img[np.abs(self.logodds) < 1e-6] = 0.62
        return img

    def explored_fraction(self):
        return float((np.abs(self.logodds) > 0.5).mean())


# ---------------------------------------------------------------------------
# Simulation loop
# ---------------------------------------------------------------------------

class Simulation:
    DT = 0.1  # s, 10 Hz control loop (same rate as the ROS node)

    def __init__(self, seed=0):
        self.rng = np.random.default_rng(seed)
        self.segs = build_world()
        self.robot = DiffDriveRobot()
        self.lidar = Lidar(rng=self.rng)
        self.ctrl = ExplorerController(seed=seed)
        self.grid = OccupancyGrid()
        self.manual = False
        self.cmd_v = self.cmd_w = 0.0
        self.state = "cruise"
        self.path = [(self.robot.x, self.robot.y)]
        self.t = 0.0
        self.last_scan = None

    def reset(self):
        self.robot.reset()
        self.grid.reset()
        self.ctrl = ExplorerController(seed=int(self.rng.integers(1 << 30)))
        self.path = [(self.robot.x, self.robot.y)]
        self.t = 0.0
        self.cmd_v = self.cmd_w = 0.0

    def tick(self):
        r = self.robot
        ranges, angles = self.lidar.scan(r.x, r.y, r.th, self.segs)
        self.last_scan = (ranges, angles)
        self.grid.update(r.x, r.y, r.th, ranges, angles, self.lidar.max_range)
        if self.manual:
            v, w, self.state = self.cmd_v, self.cmd_w, "manual"
        else:
            v, w, self.state = self.ctrl.compute(ranges, angles)
        r.step(v, w, self.DT, self.segs)
        self.path.append((r.x, r.y))
        self.t += self.DT


def run_headless(steps, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sim = Simulation()
    for _ in range(steps):
        sim.tick()
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.4))
    draw_static(axes, sim)
    fig.suptitle(f"After {sim.t:.0f}s simulated: {100*sim.grid.explored_fraction():.0f}% of the "
                 f"apartment mapped, {sim.robot.odom_dist:.1f} m driven, {sim.robot.bumps} bumps")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"Saved {out}")
    print(f"explored={sim.grid.explored_fraction():.3f} distance={sim.robot.odom_dist:.1f}m "
          f"bumps={sim.robot.bumps}")


def draw_static(axes, sim):
    ax_w, ax_m = axes
    for s in sim.segs:
        ax_w.plot([s[0], s[2]], [s[1], s[3]], color="#2b2b2b", lw=2)
    p = np.array(sim.path)
    ax_w.plot(p[:, 0], p[:, 1], color="#d9822b", lw=1)
    ax_w.add_patch(_robot_patch(sim.robot))
    ax_w.set_title("Ground truth (what Gazebo simulates)")
    ax_m.imshow(sim.grid.image(), cmap="gray", origin="lower", vmin=0, vmax=1,
                extent=[0, WORLD_W, 0, WORLD_H])
    ax_m.plot(p[:, 0], p[:, 1], color="#d9822b", lw=1)
    ax_m.set_title("Map built from LiDAR (what /map looks like)")
    for a in axes:
        a.set_xlim(-0.2, WORLD_W + 0.2)
        a.set_ylim(-0.2, WORLD_H + 0.2)
        a.set_aspect("equal")
        a.set_xticks([]); a.set_yticks([])


def _robot_patch(robot):
    from matplotlib.patches import Circle
    return Circle((robot.x, robot.y), robot.RADIUS, color="#2f6fdb", zorder=5)


def run_interactive():
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Circle

    # free up keys that matplotlib uses by default
    for k in ("keymap.back", "keymap.forward", "keymap.save", "keymap.home",
              "keymap.pan", "keymap.zoom", "keymap.quit", "keymap.fullscreen"):
        if k in matplotlib.rcParams:
            matplotlib.rcParams[k] = []

    sim = Simulation()
    fig, (ax_w, ax_m) = plt.subplots(1, 2, figsize=(13, 5.8))
    try:
        fig.canvas.manager.set_window_title("Home Robot Starter - 2D demo")
    except Exception:
        pass

    for s in sim.segs:
        ax_w.plot([s[0], s[2]], [s[1], s[3]], color="#2b2b2b", lw=2.2)
    rays = LineCollection([], colors="#e5484d", linewidths=0.5, alpha=0.35)
    ax_w.add_collection(rays)
    hits = ax_w.scatter([], [], s=4, color="#e5484d", zorder=4)
    trail_w, = ax_w.plot([], [], color="#d9822b", lw=1)
    body = Circle((0, 0), sim.robot.RADIUS, color="#2f6fdb", zorder=5)
    ax_w.add_patch(body)
    heading, = ax_w.plot([], [], color="white", lw=2, zorder=6)
    ax_w.set_title("Ground truth (what Gazebo simulates)")

    map_img = ax_m.imshow(sim.grid.image(), cmap="gray", origin="lower", vmin=0, vmax=1,
                          extent=[0, WORLD_W, 0, WORLD_H], interpolation="nearest")
    trail_m, = ax_m.plot([], [], color="#d9822b", lw=1)
    dot_m, = ax_m.plot([], [], "o", color="#2f6fdb", ms=6)
    ax_m.set_title("Map built from LiDAR (what /map looks like)")

    for a in (ax_w, ax_m):
        a.set_xlim(-0.2, WORLD_W + 0.2)
        a.set_ylim(-0.2, WORLD_H + 0.2)
        a.set_aspect("equal")
        a.set_xticks([]); a.set_yticks([])

    status = fig.text(0.5, 0.02, "", ha="center", family="monospace", fontsize=10)
    fig.text(0.5, 0.955, "m: auto/manual   arrows: drive   space: stop   r: reset   "
             "p: save map   q: quit", ha="center", fontsize=9, color="#555")

    def on_key(ev):
        if ev.key == "m":
            sim.manual = not sim.manual
            sim.cmd_v = sim.cmd_w = 0.0
        elif ev.key == "r":
            sim.reset()
        elif ev.key == "p":
            plt.imsave("map.png", sim.grid.image()[::-1], cmap="gray", vmin=0, vmax=1)
            print("Saved map.png")
        elif ev.key == "q":
            plt.close(fig)
        elif sim.manual:
            if ev.key == "up":
                sim.cmd_v = min(sim.cmd_v + 0.1, 0.5)
            elif ev.key == "down":
                sim.cmd_v = max(sim.cmd_v - 0.1, -0.3)
            elif ev.key == "left":
                sim.cmd_w = min(sim.cmd_w + 0.4, 1.6)
            elif ev.key == "right":
                sim.cmd_w = max(sim.cmd_w - 0.4, -1.6)
            elif ev.key == " ":
                sim.cmd_v = sim.cmd_w = 0.0

    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_frame):
        sim.tick()
        r = sim.robot
        ranges, angles = sim.last_scan
        ang = r.th + angles
        ex, ey = r.x + ranges * np.cos(ang), r.y + ranges * np.sin(ang)
        rays.set_segments([[(r.x, r.y), (a, b)] for a, b in zip(ex[::3], ey[::3])])
        hits.set_offsets(np.c_[ex, ey])
        p = np.array(sim.path[-3000:])
        trail_w.set_data(p[:, 0], p[:, 1])
        trail_m.set_data(p[:, 0], p[:, 1])
        body.center = (r.x, r.y)
        heading.set_data([r.x, r.x + 0.22 * math.cos(r.th)], [r.y, r.y + 0.22 * math.sin(r.th)])
        dot_m.set_data([r.x], [r.y])
        map_img.set_data(sim.grid.image())
        mode = "MANUAL" if sim.manual else "AUTO"
        status.set_text(
            f"[{mode:6}] t={sim.t:6.1f}s  /cmd_vel v={r.v:+.2f} m/s w={r.w:+.2f} rad/s  "
            f"state={sim.state:8}  mapped={100*sim.grid.explored_fraction():4.1f}%  "
            f"driven={r.odom_dist:5.1f} m")
        return rays, hits, trail_w, trail_m, body, heading, dot_m, map_img, status

    anim = FuncAnimation(fig, update, interval=int(Simulation.DT * 1000 / 2),
                         blit=False, cache_frame_data=False)
    fig.subplots_adjust(left=0.02, right=0.98, top=0.9, bottom=0.08, wspace=0.05)
    plt.show()
    return anim


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--headless", action="store_true", help="run without a window and save a PNG")
    ap.add_argument("--steps", type=int, default=2500, help="steps for --headless (10 steps = 1 s)")
    ap.add_argument("--out", default="demo_result.png", help="output image for --headless")
    args = ap.parse_args()
    if args.headless:
        run_headless(args.steps, args.out)
    else:
        run_interactive()


if __name__ == "__main__":
    sys.exit(main())
