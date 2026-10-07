#!/usr/bin/env python3
"""
Home Robot Starter - "find it, bring it, bring it back" demo
============================================================

Builds on sim_demo.py (same robot, LiDAR and occupancy map) and adds the
pieces a delivery robot for an office (or a home: --world home) needs:

  * a forward-facing camera + object detector (like a YOLO node publishing
    vision_msgs/Detection2DArray): it only sees what is inside its field of
    view, within range, and not hidden behind walls or furniture
  * a semantic map: every object the detector has confirmed, with an
    estimated position and confidence ("the keys are on the coffee table")
  * a path planner on the robot's own map (like Nav2's planner + costmap)
  * a search strategy: if the robot doesn't know where something is, it
    drives to good viewpoints and looks around until it finds it
  * a gripper: drive up to the object, face it, pick it up, take it to you,
    a colleague, a room or a desk - and later take it back where it came from

You give it commands in plain English, for example:
    take this to Diana                   bring the contract back      (later, when you want it)
    bring me the stapler                 return the stapler
    put the printout on Bob's desk       take the folder to Diana and bring it back
    where is the laptop?                 go to the meeting room       stop

Run:
    python fetch_demo.py                       # office (default)
    python fetch_demo.py --world home          # the apartment from sim_demo.py
    python fetch_demo.py --say "bring me the stapler" --say "return the stapler"
    python fetch_demo.py --gif fetch.gif --say "take this to Diana" --say "bring the contract back"
    python fetch_demo.py --headless --say "bring me the laptop"

Keys (when you are not typing in the command box):
    1-6      fetch the first six objects (shown in the panel on the right)
    e        explore everywhere and learn where everything is
    h        come back to me          c   cancel
    + / -    simulation speed         r   reset      q   quit
"""

import argparse
import heapq
import math
import re
import sys
from collections import deque

import numpy as np

from sim_demo import (WORLD_H, WORLD_W, DiffDriveRobot, Lidar, OccupancyGrid, box,
                      build_world, point_segment_dist)

# ---------------------------------------------------------------------------
# The home (--world home): furniture, rooms, objects, and people.
# The office further down uses the same settings; use_world() picks one.
# ---------------------------------------------------------------------------

# name: (x0, y0, x1, y1, height m) - must match the boxes in build_world()
FURNITURE = {
    "sofa":            (1.0, 1.0, 2.6, 1.8, 0.45),
    "coffee table":    (1.6, 3.0, 2.4, 3.6, 0.45),
    "bed":             (7.4, 0.6, 9.4, 2.2, 0.55),
    "kitchen table":   (5.0, 6.0, 6.2, 7.0, 0.75),
    "kitchen counter": (8.8, 5.0, 9.6, 7.6, 0.90),
    "wardrobe":        (0.6, 6.4, 1.2, 7.4, 2.00),
}
WALL_H = 2.4

ROOMS = [("living room", (0, 0, 4, 5)), ("hallway", (0, 5, 4, 8)),
         ("bedroom", (4, 0, 10, 4.5)), ("kitchen", (4, 4.5, 10, 8))]
ROOM_RECTS = dict(ROOMS)

# Room labels on the saved map: a spot in each room the robot can drive to, and
# the words people use for it. On a real robot you mark these once on the map
# from SLAM (e.g. by clicking them in RViz) and store them in a YAML file.
ROOM_SPOTS = {"living room": (3.0, 2.5), "hallway": (2.6, 6.6),
              "bedroom": (6.0, 2.6), "kitchen": (7.4, 6.0)}
ROOM_WORDS = {
    "living room": ("living room", "lounge", "sitting room", "salon", "tv room"),
    "hallway":     ("hallway", "hall", "corridor", "entrance", "entryway"),
    "bedroom":     ("bedroom", "bed room"),
    "kitchen":     ("kitchen",),
}

# Everything the object detector knows how to recognise (name: words people use)
CLASSES = {
    "cup":     ("cup", "mug", "coffee", "tea"),
    "keys":    ("keys", "key"),
    "remote":  ("remote", "remote control", "tv remote", "clicker"),
    "book":    ("book", "novel"),
    "phone":   ("phone", "mobile", "smartphone", "cellphone"),
    "shoes":   ("shoes", "shoe", "slippers", "sneakers"),
    "glasses": ("glasses", "spectacles", "sunglasses"),
    "wallet":  ("wallet", "purse"),
}

# What is actually lying around the apartment: name, x, y, colour, (width, height) m
OBJECTS = [
    ("cup",    5.60, 6.12, "#e5484d", (0.10, 0.12)),
    ("keys",   2.28, 3.45, "#f5a623", (0.10, 0.04)),
    ("remote", 2.40, 1.68, "#7b61ff", (0.20, 0.04)),
    ("book",   7.70, 2.05, "#12a594", (0.24, 0.05)),
    ("phone",  8.93, 6.80, "#3e63dd", (0.15, 0.03)),
    ("shoes",  2.00, 7.60, "#a1662f", (0.30, 0.12)),
    ("wallet", 2.95, 3.90, "#c2298a", (0.12, 0.03)),
]

# People the robot can deliver to, and where they are. "you" is whoever gives the
# commands. On a real robot these come from a person detector, or from each
# person's usual spot ("Mom is usually in the kitchen") - change them freely.
PEOPLE = {"you": (3.3, 4.2), "mom": (7.5, 7.3), "dad": (5.3, 1.4)}
PEOPLE_WORDS = {
    "you": ("me", "myself", "you", "us"),
    "mom": ("mom", "mum", "mother", "mommy", "mama", "mummy"),
    "dad": ("dad", "father", "daddy", "papa"),
}
HELD_AT_START = {"wallet": "you"}   # you are holding your wallet: try "take this to Mom"
HOME_POSE = (2.0, 2.4, 0.0)
EXAMPLES = "'bring me the cup', 'take the cup to the kitchen', 'give Mom my keys'"

HOME = dict(WORLD_W=WORLD_W, WORLD_H=WORLD_H, WALLS=None, FURNITURE=FURNITURE, ROOMS=ROOMS,
            ROOM_SPOTS=ROOM_SPOTS, ROOM_WORDS=ROOM_WORDS, CLASSES=CLASSES, OBJECTS=OBJECTS,
            PEOPLE=PEOPLE, PEOPLE_WORDS=PEOPLE_WORDS, HELD_AT_START=HELD_AT_START,
            HOME_POSE=HOME_POSE, EXAMPLES=EXAMPLES,
            DEFAULT_SAY=["bring me the cup", "bring me my keys"])

# ---------------------------------------------------------------------------
# The office (default): an open-plan area with desks, a corridor, a meeting
# room, the manager's office, a pantry, a print room and reception.
#
#   y=9 +---------------+-----------+------------------------+
#       |  MEETING ROOM |  MANAGER  |        PANTRY          |
#   5.4 +-----door------+---door----+------door--------------+
#       |                    CORRIDOR                         |
#   4.0 +----door--------door------+-door-+---door----------+
#       |  OPEN OFFICE (desks)     | PRINT|   RECEPTION     |
#   y=0 +--------------------------+------+-----------------+
#      x=0                        9     11.5               14
# ---------------------------------------------------------------------------

OFFICE = dict(
    WORLD_W=14.0, WORLD_H=9.0,
    WALLS=[
        (0, 0, 14, 0), (14, 0, 14, 9), (14, 9, 0, 9), (0, 9, 0, 0),           # outside
        (0, 5.4, 3.6, 5.4), (4.6, 5.4, 6.0, 5.4), (7.0, 5.4, 9.8, 5.4),      # top rooms,
        (10.8, 5.4, 14, 5.4),                                                # with doors
        (5.0, 5.4, 5.0, 9.0), (8.5, 5.4, 8.5, 9.0),
        (0, 4.0, 2.0, 4.0), (3.2, 4.0, 6.0, 4.0), (7.2, 4.0, 9.8, 4.0),      # bottom rooms
        (10.8, 4.0, 12.2, 4.0), (13.4, 4.0, 14, 4.0),
        (9.0, 0, 9.0, 4.0), (11.5, 0, 11.5, 4.0),
    ],
    FURNITURE={
        "alice's desk":    (0.6, 0.3, 2.2, 1.0, 0.75),
        "bob's desk":      (3.4, 0.3, 5.0, 1.0, 0.75),
        "my desk":         (0.6, 2.2, 2.2, 2.8, 0.75),
        "carol's desk":    (5.8, 2.0, 7.6, 2.7, 0.75),
        "meeting table":   (1.2, 6.6, 3.8, 7.8, 0.75),
        "manager's desk":  (5.9, 7.5, 7.9, 8.3, 0.75),
        "pantry table":    (9.6, 6.6, 11.0, 7.6, 0.75),
        "pantry counter":  (12.8, 5.8, 13.6, 8.6, 0.90),
        "printer":         (9.3, 0.3, 10.3, 1.1, 1.00),
        "supply shelf":    (10.9, 0.3, 11.3, 2.5, 1.80),
        "reception desk":  (12.0, 1.6, 13.6, 2.2, 1.00),
    },
    ROOMS=[("meeting room", (0, 5.4, 5, 9)), ("manager's office", (5, 5.4, 8.5, 9)),
           ("pantry", (8.5, 5.4, 14, 9)), ("corridor", (0, 4.0, 14, 5.4)),
           ("open office", (0, 0, 9, 4.0)), ("print room", (9, 0, 11.5, 4.0)),
           ("reception", (11.5, 0, 14, 4.0))],
    ROOM_SPOTS={"meeting room": (4.3, 6.6), "manager's office": (6.5, 6.2),
                "pantry": (11.9, 6.2), "corridor": (7.0, 4.7), "open office": (3.0, 1.7),
                "print room": (10.3, 2.2), "reception": (12.6, 3.2)},
    ROOM_WORDS={
        "meeting room":     ("meeting room", "conference room", "boardroom", "board room"),
        "manager's office": ("manager's office", "diana's office", "boss's office",
                             "managers office"),
        "pantry":           ("pantry", "kitchen", "break room", "kitchenette", "coffee room"),
        "corridor":         ("corridor", "hallway", "hall"),
        "open office":      ("open office", "open space", "open plan", "work area", "desks"),
        "print room":       ("print room", "printer room", "copy room"),
        "reception":        ("reception", "front desk", "lobby", "entrance"),
    },
    CLASSES={
        "stapler":  ("stapler",),
        "laptop":   ("laptop", "computer", "macbook"),
        "printout": ("printout", "printouts", "print-out", "document", "documents", "papers",
                     "report", "prints"),
        "mug":      ("mug", "cup", "coffee", "tea"),
        "keys":     ("keys", "key"),
        "charger":  ("charger", "cable", "adapter", "power brick"),
        "folder":   ("folder", "file", "binder"),
        "parcel":   ("parcel", "package", "delivery", "box"),
        "contract": ("contract", "agreement", "form"),
        "scissors": ("scissors",),
    },
    OBJECTS=[
        ("stapler",  3.80, 0.88, "#e5484d", (0.16, 0.06)),
        ("laptop",   3.65, 6.75, "#3e63dd", (0.32, 0.03)),
        ("printout", 9.80, 0.98, "#12a594", (0.21, 0.02)),
        ("mug",     10.85, 6.72, "#a1662f", (0.09, 0.11)),
        ("keys",    12.40, 2.08, "#f5a623", (0.10, 0.04)),
        ("charger",  6.00, 2.58, "#7b61ff", (0.10, 0.04)),
        ("folder",   7.00, 7.62, "#d6409f", (0.24, 0.04)),
        ("parcel",  13.40, 3.50, "#8d6e4a", (0.30, 0.25)),
        ("contract", 1.00, 3.00, "#30a46c", (0.21, 0.02)),
    ],
    PEOPLE={"you": (1.4, 3.3), "alice": (1.4, 1.5), "bob": (4.2, 1.5), "carol": (6.7, 3.3),
            "diana": (6.9, 6.9), "eve": (13.0, 2.9)},
    PEOPLE_WORDS={
        "you":   ("me", "myself", "you", "us"),
        "alice": ("alice",),
        "bob":   ("bob",),
        "carol": ("carol",),
        "diana": ("diana", "manager", "boss"),
        "eve":   ("eve", "receptionist"),
    },
    HELD_AT_START={"contract": "you"},    # you have a contract: "take this to Diana"
    HOME_POSE=(3.0, 3.0, 0.0),            # the robot's charging dock, next to your desk
    EXAMPLES="'take this to Diana', 'bring the contract back', 'bring me the stapler', "
             "'return the stapler'",
    DEFAULT_SAY=["take this to Diana", "bring me the stapler", "bring the contract back"],
)

WORLDS = {"home": HOME, "office": OFFICE}


def use_world(name):
    """Switch every setting above (rooms, furniture, people, objects...) to one world."""
    cfg = WORLDS[name]
    globals().update(cfg)
    globals()["ROOM_RECTS"] = dict(cfg["ROOMS"])
    globals()["PERSON"] = cfg["PEOPLE"]["you"]


def make_walls():
    """Wall + furniture segments for the LiDAR, camera and collisions."""
    if WALLS is None:
        return build_world()                     # the apartment from sim_demo.py
    segs = list(WALLS)
    for x0, y0, x1, y1, _h in FURNITURE.values():
        segs += box(x0, y0, x1, y1)
    return np.array(segs, dtype=float)


def who(person):
    return "you" if person == "you" else person.capitalize()


def the(furniture):
    """How the robot names a piece of furniture: 'Bob's desk', 'your desk', 'the printer'."""
    first, _, rest = furniture.partition(" ")
    if first == "my":
        return "your " + rest
    if first.endswith("'s") and first[:-2] in PEOPLE_WORDS:
        return furniture[0].upper() + furniture[1:]
    return "the " + furniture


use_world("office")


def room_of(x, y):
    for name, (x0, y0, x1, y1) in ROOMS:
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return "home"


def furniture_at(x, y):
    for name, (x0, y0, x1, y1, _h) in FURNITURE.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return None


def drop_points(furniture, inset=0.12, step=0.1):
    """Spots on top of a piece of furniture, just inside its edge, where the arm can put
    something down."""
    x0, y0, x1, y1, _h = FURNITURE[furniture]
    x0, y0, x1, y1 = x0 + inset, y0 + inset, x1 - inset, y1 - inset
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    pts = [(x, y0) for x in xs] + [(x, y1) for x in xs] + \
          [(x0, y) for y in ys] + [(x1, y) for y in ys]
    return np.array(pts)


PLURAL = {"keys", "shoes", "glasses"}


def is_(name):
    return "are" if name in PLURAL else "is"


def it(name):
    return "them" if name in PLURAL else "it"


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def raycast(x, y, ang, segs):
    """Distance and index of the first segment hit by each ray (absolute angles)."""
    dx, dy = np.cos(ang)[:, None], np.sin(ang)[:, None]
    x1, y1, x2, y2 = (segs[:, i][None, :] for i in range(4))
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((x1 - x) * ey - (y1 - y) * ex) / den
        u = ((x1 - x) * dy - (y1 - y) * dx) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(hit, t, np.inf)
    idx = t.argmin(axis=1)
    return t[np.arange(len(ang)), idx], idx


def line_of_sight(px, py, qx, qy, segs):
    """True if the straight line p->q crosses none of the segments."""
    if len(segs) == 0:
        return True
    x1, y1, x2, y2 = segs.T
    dx, dy = qx - px, qy - py
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((x1 - px) * ey - (y1 - py) * ex) / den
        u = ((x1 - px) * dy - (y1 - py) * dx) / den
    hit = (np.abs(den) > 1e-12) & (t > 0) & (t < 1) & (u >= 0) & (u <= 1)
    return not hit.any()


class WorldObject:
    def __init__(self, name, x, y, color, size, segs):
        self.name, self.x, self.y, self.color, self.size = name, x, y, color, size
        self.carried = False
        self.holder = None          # person holding it ("you", "mom", ...) or None
        self.origin = None          # where it was before the robot last moved it
        self.held_since = 0.0
        self.place(x, y, segs)

    def place(self, x, y, segs):
        self.x, self.y = x, y
        self.host = furniture_at(x, y)
        self.z = FURNITURE[self.host][4] if self.host else 0.0
        # the object sits on top of its host furniture, so that furniture's
        # edges don't hide it from the camera
        if self.host:
            hx0, hy0, hx1, hy1, _ = FURNITURE[self.host]
            host = np.array(box(hx0, hy0, hx1, hy1))
            self.occluders = np.array([s for s in segs
                                       if not any(np.allclose(s, h) for h in host)])
        else:
            self.occluders = segs

    def where(self):
        if self.holder:
            return f"with {who(self.holder)}"
        place = f"on {the(self.host)}" if self.host else "on the floor"
        return f"{place} in the {room_of(self.x, self.y)}"


# ---------------------------------------------------------------------------
# Camera + object detector
# ---------------------------------------------------------------------------

class Camera:
    FOV = math.radians(80)
    RANGE = 3.5          # m - beyond this the detector is unreliable
    HEIGHT = 1.0         # m - camera on a mast, so it can see table tops

    def __init__(self, rng):
        self.rng = rng

    def visible(self, robot, obj):
        """Is obj inside the camera frustum and not occluded? -> (dist, bearing) or None"""
        if obj.carried:
            return None
        dx, dy = obj.x - robot.x, obj.y - robot.y
        d = math.hypot(dx, dy)
        if d > self.RANGE or d < 0.05:
            return None
        b = wrap(math.atan2(dy, dx) - robot.th)
        if abs(b) > self.FOV / 2:
            return None
        if not line_of_sight(robot.x, robot.y, obj.x, obj.y, obj.occluders):
            return None
        return d, b

    def detect(self, robot, objects):
        """Simulated detector output: list of (name, score, est_x, est_y)."""
        out = []
        for obj in objects:
            vis = self.visible(robot, obj)
            if vis is None:
                continue
            d, _ = vis
            p_detect = 0.95 if d < 2.0 else 0.95 - 0.35 * (d - 2.0) / (self.RANGE - 2.0)
            if self.rng.random() > p_detect:
                continue                                       # missed this frame
            score = float(np.clip(0.97 - 0.06 * d + self.rng.normal(0, 0.03), 0.35, 0.99))
            sd = 0.02 + 0.015 * d                              # depth gets noisier far away
            out.append((obj.name, score, obj.x + self.rng.normal(0, sd),
                        obj.y + self.rng.normal(0, sd)))
        return out


class Memory:
    """Semantic map: what the robot has seen and where."""

    CONFIRM_HITS = 3

    def __init__(self):
        self.items = {}

    def add(self, name, score, x, y, t):
        m = self.items.setdefault(name, dict(x=x, y=y, hits=0, score=score, t=t))
        k = min(m["hits"], 10)
        m["x"] = (m["x"] * k + x) / (k + 1)
        m["y"] = (m["y"] * k + y) / (k + 1)
        m["score"] = (m["score"] * k + score) / (k + 1)
        m["hits"] += 1
        m["t"] = t
        return m["hits"] == self.CONFIRM_HITS          # newly confirmed?

    def known(self, name):
        m = self.items.get(name)
        return m if m and m["hits"] >= self.CONFIRM_HITS else None

    def set(self, name, x, y, t):
        self.items[name] = dict(x=x, y=y, hits=self.CONFIRM_HITS, score=0.99, t=t)

    def forget(self, name):
        self.items.pop(name, None)


# ---------------------------------------------------------------------------
# Planner: costmap + Dijkstra on the robot's own map (what Nav2 does)
# ---------------------------------------------------------------------------

def dilate(mask, r):
    out = mask.copy()
    ny, nx = mask.shape
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy > r * r or (dx == 0 and dy == 0):
                continue
            out[max(dy, 0):ny + min(dy, 0), max(dx, 0):nx + min(dx, 0)] |= \
                mask[max(-dy, 0):ny + min(-dy, 0), max(-dx, 0):nx + min(-dx, 0)]
    return out


def box_sum(a, r):
    """Sum of a over a (2r+1)x(2r+1) window around every cell."""
    ny, nx = a.shape
    s = np.zeros((ny + 1, nx + 1))
    s[1:, 1:] = np.cumsum(np.cumsum(a, 0), 1)
    y0 = np.clip(np.arange(ny) - r, 0, ny); y1 = np.clip(np.arange(ny) + r + 1, 0, ny)
    x0 = np.clip(np.arange(nx) - r, 0, nx); x1 = np.clip(np.arange(nx) + r + 1, 0, nx)
    return (s[y1][:, x1] - s[y0][:, x1] - s[y1][:, x0] + s[y0][:, x0])


class Planner:
    RES = 0.1
    NEIGH = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

    def __init__(self):
        self.nx, self.ny = int(WORLD_W / self.RES), int(WORLD_H / self.RES)
        self.viewed = np.zeros((self.ny, self.nx), dtype=bool)   # seen by the camera
        self.update_costmap(None)

    def reset(self):
        self.viewed[:] = False
        self.update_costmap(None)

    def cell(self, x, y):
        return (min(max(int(y / self.RES), 0), self.ny - 1),
                min(max(int(x / self.RES), 0), self.nx - 1))

    def centre(self, j, i):
        return ((i + 0.5) * self.RES, (j + 0.5) * self.RES)

    def update_costmap(self, grid):
        if grid is None:
            lo = np.zeros((self.ny * 2, self.nx * 2), dtype=np.float32)
        else:
            lo = grid.logodds
        lo = lo[:self.ny * 2, :self.nx * 2].reshape(self.ny, 2, self.nx, 2)
        occ = (lo > 0.6).any(axis=(1, 3))
        known = (np.abs(lo) > 0.5).any(axis=(1, 3))
        self.free = known & ~occ
        self.core = dilate(occ, 1)          # touching an obstacle: never go there
        self.lethal = dilate(occ, 3)        # robot body would scrape: avoid hard
        soft = dilate(occ, 5)               # keep a comfortable distance if possible
        cost = np.ones((self.ny, self.nx))
        cost[~known] += 2.0                 # optimistic about unknown space, but prefer known
        cost[soft] += 2.0
        cost[self.lethal] += 40.0
        cost[self.core] = np.inf
        self.cost = cost

    def dijkstra(self, x, y):
        nx, ny = self.nx, self.ny
        cost = self.cost.ravel().tolist()
        n = nx * ny
        dist = [math.inf] * n
        parent = [-1] * n
        sj, si = self.cell(x, y)
        s = sj * nx + si
        dist[s] = 0.0
        heap = [(0.0, s)]
        while heap:
            d, k = heapq.heappop(heap)
            if d > dist[k]:
                continue
            j, i = divmod(k, nx)
            for dj, di, step in self.NEIGH:
                jj, ii = j + dj, i + di
                if 0 <= jj < ny and 0 <= ii < nx:
                    kk = jj * nx + ii
                    nd = d + step * cost[kk]
                    if nd < dist[kk]:
                        dist[kk] = nd
                        parent[kk] = k
                        heapq.heappush(heap, (nd, kk))
        return np.array(dist).reshape(ny, nx), parent

    def path_to(self, parent, j, i):
        k = j * self.nx + i
        cells = []
        while k != -1:
            cells.append(divmod(k, self.nx))
            k = parent[k]
        return [self.centre(jj, ii) for jj, ii in reversed(cells)]

    def plan(self, x, y, goal_mask, extra_cost=None):
        """Cheapest path from (x, y) to any cell in goal_mask -> (path, (j, i)) or None."""
        dist, parent = self.dijkstra(x, y)
        score = np.where(goal_mask, dist, np.inf)
        if extra_cost is not None:
            score = score + extra_cost
        k = int(np.argmin(score))
        if not np.isfinite(score.flat[k]):
            return None
        j, i = divmod(k, self.nx)
        path = self.path_to(parent, j, i)
        path[0] = (x, y)
        return path, (j, i)

    def ring(self, x, y, r_min, r_max):
        """Cells at distance [r_min, r_max] from (x, y) - where to stand to reach it."""
        jj, ii = np.mgrid[0:self.ny, 0:self.nx]
        cx, cy = (ii + 0.5) * self.RES, (jj + 0.5) * self.RES
        d = np.hypot(cx - x, cy - y)
        return (d >= r_min) & (d <= r_max)

    def mark_viewed(self, x, y, th, ranges, angles, fov, cam_range):
        m = np.abs(angles) <= fov / 2
        ang = th + angles[m]
        r = np.minimum(ranges[m], cam_range)
        d = np.arange(0, cam_range, self.RES * 0.5)[None, :]
        ok = d < r[:, None]
        px = (x + d * np.cos(ang)[:, None])[ok]
        py = (y + d * np.sin(ang)[:, None])[ok]
        i = np.clip((px / self.RES).astype(int), 0, self.nx - 1)
        j = np.clip((py / self.RES).astype(int), 0, self.ny - 1)
        self.viewed[j, i] = True

    def unsearched(self):
        """Free floor the camera hasn't looked at yet (where a lost thing could be)."""
        return self.free & ~self.viewed

    def next_viewpoint(self, x, y, blacklist, region=None):
        """Pick where to go next to look for things: close, and with a lot unsearched nearby.
        region=(x0, y0, x1, y1) limits the search to one room."""
        jj, ii = np.mgrid[0:self.ny, 0:self.nx]
        cx, cy = (ii + 0.5) * self.RES, (jj + 0.5) * self.RES
        todo = self.unsearched()
        inside = np.ones_like(todo)
        if region is not None:
            x0, y0, x1, y1 = region
            inside = (cx > x0 + 0.15) & (cx < x1 - 0.15) & (cy > y0 + 0.15) & (cy < y1 - 0.15)
            todo = todo & inside
        gain = box_sum(todo.astype(float), 15)              # unsearched cells within 1.5 m
        ok = self.free & ~self.lethal & inside & (gain >= 25)
        for bx, by in blacklist:
            ok &= np.hypot(cx - bx, cy - by) > 0.9
        if not ok.any():
            return None
        return self.plan(x, y, ok, extra_cost=-0.12 * gain)


# ---------------------------------------------------------------------------
# Understanding commands
# ---------------------------------------------------------------------------

def find_class(text):
    for furniture in FURNITURE:                 # "coffee table" is a place, not a coffee cup
        text = text.replace(furniture, " ")
    for words in ROOM_WORDS.values():           # "print room" is a place, not a printout
        for w in words:
            text = re.sub(r"\b" + re.escape(w) + r"\b", " ", text)
    for name, words in CLASSES.items():
        for w in words:
            if re.search(r"\b" + re.escape(w) + r"\b", text):
                return name
    return None


NOT_PLACES = {"floor", "ground", "house", "home", "way", "room", "front", "end", "top"}
PUT_VERBS = r"\b(put|place|leave|set|drop|store)\b"
MAX_PLACE_HEIGHT = 1.0      # m - the arm can't put things on top of the wardrobe


def place_at(s):
    """Place named at the very start of s -> ('room', name) / ('furniture', name) / None.
    The longest match wins, so 'bedroom' isn't read as 'bed'."""
    best = None
    names = [(w, ("room", r)) for r, ws in ROOM_WORDS.items() for w in ws]
    names += [(f, ("furniture", f)) for f in FURNITURE]
    for w, place in names:
        if s.startswith(w) and not s[len(w):len(w) + 1].isalnum():
            if best is None or len(w) > len(best[0]):
                best = (w, place)
    return best[1] if best else None


def person_at(s):
    for person, words in PEOPLE_WORDS.items():
        for w in words:
            if re.match(re.escape(w) + r"\b", s):
                return person
    return None


def place_room(place):
    """The room a place is in."""
    if place[0] == "room":
        return place[1]
    x0, y0, x1, y1, _h = FURNITURE[place[1]]
    return room_of((x0 + x1) / 2, (y0 + y1) / 2)


def find_places(t):
    """Where to pick something up and where to put it.
    'move the cup from the kitchen table to the bed' -> (('furniture', 'kitchen table'),
                                                          ('furniture', 'bed'))
    Each is None, a place, ('person', None), or ('?', word) for a place not on the map."""
    src = dest = None
    phrases = []
    for m in re.finditer(r"\b(from|in|on|at|inside|to|into|onto|for)\s+(the\s+|my\s+|our\s+)?", t):
        prep, article, rest = m.group(1), m.group(2), t[m.end():]
        full = t[m.start(2):] if article else rest        # "my desk" is a place name
        place = place_at(full) or place_at(rest)
        person = person_at(rest) if place is None and prep in ("to", "for") else None
        if person:
            phrases.append(("dest", ("person", person)))
            continue
        if place is None:
            w = re.match(r"\w+(\s+room)?", rest)
            if not (article and w) or w.group(0) in NOT_PLACES or prep == "for":
                continue
            place = ("?", w.group(0))
        phrases.append((prep, place))
    # "give Mom the cup", "bring dad my keys"
    m = re.search(r"\b(?:bring|give|hand|send|take)\s+(?:my\s+)?(\w+)\s+(?:the|my|a|an|his|her|this|that|some)\b", t)
    if m and person_at(m.group(1)):
        phrases.append(("dest", ("person", person_at(m.group(1)))))
    explicit_dest = any(p in ("to", "into", "onto", "dest") for p, _ in phrases)
    putting = re.search(PUT_VERBS, t)
    for k, (prep, place) in enumerate(phrases):
        if k and phrases[k - 1][1][0] == "person" and prep in ("in", "on", "at", "inside"):
            continue                             # "to Mom in the kitchen" says where Mom is
        if prep in ("to", "into", "onto", "dest"):
            dest = place
        elif prep == "from":
            src = place
        elif putting and not explicit_dest:      # "put the book ON the bed"
            dest = place
        elif src is None:                        # "the keys ON the coffee table"
            src = place
    if src is None and dest is None:             # "explore the kitchen", "the kitchen cup"
        for i in range(len(t)):
            if i == 0 or not t[i - 1].isalnum():
                src = place_at(t[i:])
                if src:
                    break
    return src, dest


ROUND_TRIP = re.compile(r"\b(and|then)\s+(bring|take|get)\s+(it|them)\s+back\b|\bwait (for|while)\b"
                        r"|\bround[- ]?trip\b|\bcome back with\b|\bget (it|them) signed\b")
RETURNING = re.compile(r"\breturn\b|\b(put|take|give|send|bring|get)\b.*\bback\b")


def parse_command(text):
    """'take this to Diana and bring it back' -> ('fetch', 'it', None, ('person', 'diana'), 'back')
    Returns (kind, target, room, destination, then)."""
    kind, arg, room, dest = _parse(text)
    t = text.lower()
    then = None
    if kind == "fetch":
        if ROUND_TRIP.search(t):
            then = "back"                        # hand it over, wait, bring it back
        elif dest is None and RETURNING.search(t):
            dest = ("origin", None)              # "return the stapler": where it came from
    return kind, arg, room, dest, then


def _parse(text):
    """'move my keys from the kitchen to the bed' -> ('fetch', 'keys', 'kitchen', ('furniture', 'bed')).
    Returns (kind, target, room, destination) or (None, reason, None, None)."""
    t = text.lower().strip()
    src, dest = find_places(t)
    if dest is None:                                 # "give the cup to John": someone we don't know
        known = {w for ws in PEOPLE_WORDS.values() for w in ws} | {"i"}
        m = re.search(r"\b(?:to|for|give|send|hand)\s+([A-Z][a-z]+)\b", text)
        if m and m.group(1).lower() not in known and not place_at(m.group(1).lower()):
            dest = ("?person", m.group(1))
    room = None
    if src:
        room = "?" + src[1] if src[0] == "?" else (place_room(src) if src[0] != "person" else None)
    if re.search(r"\b(stop|cancel|abort|never ?mind)\b", t):
        return "cancel", None, None, None
    if re.search(r"\b(where|have you seen|did you see|seen my)\b", t):
        name = find_class(t)
        return ("where", name, None, None) if name else (None, "what", None, None)
    if re.search(r"\b(explore|learn|map|scan|look around|tour)\b", t):
        return "explore", None, room, None
    if re.search(r"\b(come (here|back)|go home|come to me|go back to (your )?(dock|base))\b", t) \
            or re.fullmatch(r"\s*return\W*", t):
        return "home", None, None, None
    name = find_class(t)
    if name is None and re.search(r"\b(it|them|that|this)\b", t) and \
            re.search(r"\b(bring|get|fetch|take|put|place|leave|move|carry|give|drop|set)\b", t):
        name = "it"                              # resolved to the object we last talked about
    if name and dest is None and re.search(r"\b(put|set|drop)\b.*\bdown\b|\bdrop\b|\blet go\b", t):
        return "drop", name, None, None
    if name:
        return "fetch", name, room, dest
    if re.search(r"\b(go|drive|move|head|walk)\b", t) and (dest or src):
        place = dest or src
        if place[0] == "?":
            return "goto", None, "?" + place[1], None
        if place[0] == "person":
            if place[1] == "you":
                return "home", None, None, None
            return "goto", None, room_of(*PEOPLE[place[1]]), None
        return "goto", None, place_room(place), None
    m = re.search(r"\b(?:bring|get|fetch|find|grab|take|put|move)\s+(?:me\s+)?(?:my|the|a|an|some)?\s*(\w+)", t)
    return None, (m.group(1) if m else ""), None, None


# ---------------------------------------------------------------------------
# The robot's brain: task queue + behaviour state machine
# ---------------------------------------------------------------------------

class FetchSim:
    DT = 0.1
    REACH = 0.65           # m from robot centre the arm can grab things

    def __init__(self, seed=0):
        self.seed = seed
        self.segs = make_walls()
        self.reset()

    def reset(self):
        self.rng = np.random.default_rng(self.seed)
        self.robot = DiffDriveRobot(*HOME_POSE)
        self.lidar = Lidar(rng=self.rng)
        self.camera = Camera(self.rng)
        self.grid = OccupancyGrid(WORLD_W, WORLD_H)
        self.planner = Planner()
        self.memory = Memory()
        self.objects = [WorldObject(n, x, y, c, s, self.segs) for n, x, y, c, s in OBJECTS]
        self.tasks = deque()
        self.task = None
        self.search_room = None
        self.phase = "idle"
        self.path = None
        self.goal_cell = None
        self.blacklist = []
        self.messages = []
        self.detections = []
        self.carrying = None
        self.last_object = None
        self.drop_point = None
        self.t = 0.0
        self.tick_n = 0
        self.last_scan = None
        self.cmd = (0.0, 0.0)
        for o in self.objects:
            if o.name in HELD_AT_START:
                self.give_to(o, HELD_AT_START[o.name])

    # --- talking -----------------------------------------------------------

    def say(self, msg):
        self.messages.append((self.t, msg))
        print(f"[{self.t:6.1f}s] {msg}")

    def command(self, text):
        kind, arg, room, dest, then = parse_command(text)
        self.say(f'You: "{text.strip()}"')
        if arg == "it":
            last = self.carrying.name if self.carrying else self.last_object
            mine = sorted((o for o in self.objects if o.holder == "you"),
                          key=lambda o: o.held_since)
            if re.search(r"\b(this|these)\b", text.lower()) and mine:
                last = mine[-1].name                 # "take this to Mom": what you're holding
            if last is None:
                self.say("Robot: Which object do you mean? Try: take the cup to the kitchen")
                return
            arg = last
        if kind == "fetch":
            self.last_object = arg
        if then and not (dest and dest[0] == "person" and dest[1] != "you"):
            then = None                          # "bring it back" only makes sense for a person
        if dest and dest[0] == "?person":
            self.say(f"Robot: I don't know who {dest[1]} is. I can deliver to: "
                     f"{', '.join(who(p) for p in PEOPLE)}. I'll bring it to you instead.")
            dest = None
        if dest and dest[0] == "?":
            self.say(f"Robot: I don't know where '{dest[1]}' is. My map has the rooms "
                     f"{', '.join(ROOM_SPOTS)} and the {', '.join(FURNITURE)}. "
                     f"I'll bring it to you instead.")
            dest = None
        if dest and dest[0] == "furniture" and FURNITURE[dest[1]][4] > MAX_PLACE_HEIGHT:
            self.say(f"Robot: The top of {the(dest[1])} is too high for my arm. "
                     f"I'll bring it to you instead.")
            dest = None
        if room and room.startswith("?"):
            self.say(f"Robot: I don't know a room called '{room[1:]}'. My map has: "
                     f"{', '.join(ROOM_SPOTS)}. I'll search everywhere.")
            room = None
            if kind == "goto":
                return
        if kind is None:
            if arg == "what":
                self.say("Robot: Where is what? Try: where are my keys?")
            elif arg:
                self.say(f"Robot: Sorry, my detector can't recognise '{arg}'. "
                         f"I can recognise: {', '.join(CLASSES)}.")
            else:
                self.say("Robot: Sorry, I didn't understand. Try: bring me the cup")
            return
        if kind == "cancel":
            self.tasks.clear()
            if self.task:
                self.finish("Robot: OK, stopping.")
            if self.carrying:
                self.put_down()
            return
        if kind == "where":
            self.answer_where(arg)
            return
        if kind == "drop":
            if self.carrying and self.carrying.name == arg:
                self.tasks.clear()
                if self.task:
                    self.finish(None)
                self.put_down()
            else:
                self.say(f"Robot: I'm not holding the {arg}.")
            return
        task = (kind, arg, room, dest or (("person", "you") if kind == "fetch" else None), then)
        if self.task is None:
            self.start(task)
        else:
            self.tasks.append(task)
            self.say("Robot: Got it, I'll do that next.")

    def answer_where(self, name):
        m = self.memory.known(name)
        if not m:
            self.say(f"Robot: I haven't seen the {name} yet. Ask me to bring {it(name)} and I'll look.")
            return
        obj = self.obj(name)
        if obj and obj.holder:
            self.say(f"Robot: {who(obj.holder).capitalize()} "
                     f"{'have' if obj.holder == 'you' else 'has'} the {name}.")
            return
        where = obj.where().rsplit(" in the ", 1)[0] if obj else "somewhere"
        self.say(f"Robot: The {name} {is_(name)} {where} in the {room_of(m['x'], m['y'])} "
                 f"({100 * m['score']:.0f}% sure, seen {self.t - m['t']:.0f}s ago).")

    def obj(self, name):
        return next((o for o in self.objects if o.name == name), None)

    # --- tasks -------------------------------------------------------------

    def start(self, task):
        self.task = task
        self.task_t0 = self.t
        self.path = None
        self.blacklist = []
        kind, name, room, dest, _then = task
        self.search_room = None
        if dest == ("origin", None):
            # "return the stapler": worked out now, not when you said it, so it also
            # works right after a queued "bring me the stapler"
            obj = self.obj(name)
            if obj is None or obj.origin is None:
                self.say(f"Robot: I haven't moved the {name}, so I don't know where {it(name)} "
                         f"should go back to. Tell me where, e.g. 'put the {name} on my desk'.")
                return self.finish(None)
            dest = obj.origin
            self.task = task = task[:3] + (dest, None)
        self.dest = dest
        if kind == "fetch":
            obj = self.obj(name)
            if obj is not None and obj.holder and dest == ("person", obj.holder):
                self.say(f"Robot: {who(obj.holder).capitalize()} already "
                         f"{'have' if obj.holder == 'you' else 'has'} the {name}!")
                return self.finish(None)
            if self.carrying and self.carrying.name == name:
                self.say(f"Robot: I'm already holding the {name}. Taking {it(name)} "
                         f"{self.dest_text()}.")
                return self.set_phase("deliver")
            if self.carrying:
                self.say(f"Robot: My gripper is full, so I'll put the {self.carrying.name} "
                         f"down here first.")
                self.put_down()
            m = self.memory.known(name)
            if m and obj is not None and obj.holder:
                self.say(f"Robot: Coming to get the {name} from {who(obj.holder)}, "
                         f"then taking {it(name)} {self.dest_text()}.")
                return self.set_phase("approach")
            if m:
                seen_in = room_of(m["x"], m["y"])
                if room and room != seen_in:
                    self.say(f"Robot: I saw the {name} in the {seen_in}, not in the {room}. "
                             f"Going there.")
                else:
                    self.say(f"Robot: I know where to find the {name}. On my way.")
                return self.set_phase("approach")
            if room:
                self.search_room = room
                self.say(f"Robot: Going to the {room} to look for the {name}.")
                return self.set_phase("goto_room")
            self.say(f"Robot: I haven't seen the {name} yet. Let me look around.")
            return self.set_phase("look")
        if kind == "goto":
            self.search_room = room
            self.say(f"Robot: Going to the {room}.")
            return self.set_phase("goto_room")
        if kind == "explore":
            if room:
                self.search_room = room
                self.say(f"Robot: Going to look around the {room}.")
                return self.set_phase("goto_room")
            self.say("Robot: Exploring the home and taking note of everything I see.")
            return self.set_phase("look")
        if kind == "home":
            self.say("Robot: Coming to you.")
            return self.set_phase("return")

    def finish(self, msg):
        if msg:
            self.say(msg)
        self.task = None
        self.search_room = None
        self.drop_point = None
        self.path = None
        self.goal_cell = None
        self.phase = "idle"
        if self.tasks:
            self.start(self.tasks.popleft())

    def set_phase(self, phase, timer=0):
        self.phase = phase
        self.timer = timer
        self.spun = 0.0
        self.path = None
        self.goal_cell = None
        self.since_plan = 0
        self.stuck_ref = (self.robot.x, self.robot.y, self.tick_n)

    # --- main loop ---------------------------------------------------------

    def tick(self):
        r = self.robot
        ranges, angles = self.lidar.scan(r.x, r.y, r.th, self.segs)
        self.last_scan = (ranges, angles)
        self.grid.update(r.x, r.y, r.th, ranges, angles, self.lidar.max_range)
        self.planner.mark_viewed(r.x, r.y, r.th, ranges, angles, Camera.FOV, Camera.RANGE)
        if self.tick_n % 5 == 0:
            self.planner.update_costmap(self.grid)

        self.detections = self.camera.detect(r, self.objects)
        for name, score, ex, ey in self.detections:
            if self.memory.add(name, score, ex, ey, self.t):
                obj = self.obj(name)
                self.say(f"Robot: I see the {name} {obj.where()} ({100 * score:.0f}%).")

        if self.task is None and self.tasks:
            self.start(self.tasks.popleft())
        v, w = self.behave(ranges, angles)
        self.cmd = (v, w)
        r.step(v, w, self.DT, self.segs)
        if self.carrying:
            self.carrying.x = r.x + 0.08 * math.cos(r.th)
            self.carrying.y = r.y + 0.08 * math.sin(r.th)
        self.t += self.DT
        self.tick_n += 1

    def behave(self, ranges, angles):
        kind, name, _room, _dest, _then = self.task or (None, None, None, None, None)
        ph = self.phase

        # While searching, stop as soon as the detector has confirmed the target
        if kind == "fetch" and ph in ("look", "search", "goto_room") and self.memory.known(name):
            self.say(f"Robot: Found the {name}! Going to pick {it(name)} up.")
            self.set_phase("approach")
            ph = self.phase

        if ph == "idle":
            return 0.0, 0.0
        if ph == "look":                                  # turn on the spot, camera scanning
            w = 1.3
            self.spun += w * self.DT
            if self.spun >= 2 * math.pi:
                self.blacklist.append((self.robot.x, self.robot.y))
                self.next_search()
            return 0.0, w
        if ph in ("search", "goto_room", "approach", "deliver", "return"):
            return self.navigate(ranges, angles)
        if ph == "align":                                 # face the object before grasping
            m = self.memory.known(name)
            if not m:
                self.set_phase("look")
                return 0.0, 0.0
            e = wrap(math.atan2(m["y"] - self.robot.y, m["x"] - self.robot.x) - self.robot.th)
            if abs(e) < 0.1:
                self.say(f"Robot: Reaching for the {name}...")
                self.set_phase("grasp", timer=15)
                return 0.0, 0.0
            return 0.0, float(np.clip(2.5 * e, -1.0, 1.0))
        if ph == "grasp":
            self.timer -= 1
            if self.timer <= 0:
                self.try_grasp(name)
            return 0.0, 0.0
        if ph == "align_drop":                            # face the spot, then put it down
            px, py = self.drop_point
            e = wrap(math.atan2(py - self.robot.y, px - self.robot.x) - self.robot.th)
            if abs(e) < 0.1:
                self.set_phase("place", timer=12)
                return 0.0, 0.0
            return 0.0, float(np.clip(2.5 * e, -1.0, 1.0))
        if ph == "place":
            self.timer -= 1
            if self.timer <= 0:
                self.place_object()
            return 0.0, 0.0
        if ph == "wait_person":                           # they're signing / reading it
            self.timer -= 1
            if self.timer <= 0:
                obj = self.wait_obj
                person = obj.holder
                obj.holder, obj.carried, obj.host = None, True, None
                self.carrying = obj
                self.memory.forget(obj.name)
                self.dest = obj.origin or ("person", "you")
                self.task = self.task[:3] + (self.dest, None)
                self.say(f"Robot: {who(person).capitalize()} is done with the {obj.name}. "
                         f"Taking {it(obj.name)} back {self.dest_text()}.")
                self.set_phase("deliver")
            return 0.0, 0.0
        if ph == "handover":
            self.timer -= 1
            if self.timer <= 0:
                self.hand_over()
            return 0.0, 0.0
        if ph == "backup":
            self.timer -= 1
            if self.timer <= 0:
                self.phase, self.path = self.resume_phase, None
                self.stuck_ref = (self.robot.x, self.robot.y, self.tick_n)
            return -0.12, 0.6
        return 0.0, 0.0

    def next_search(self):
        kind, name, _room, _dest, _then = self.task
        r = self.robot
        room = self.search_room
        res = self.planner.next_viewpoint(r.x, r.y, self.blacklist,
                                          ROOM_RECTS[room] if room else None)
        if res is None and room:
            # Searched the whole room: the hint was wrong (or the thing was moved).
            # Don't give up - widen the search to the rest of the home.
            self.search_room = None
            if kind == "fetch":
                self.say(f"Robot: The {name} {is_(name)}n't in the {room}. "
                         f"I'll search the rest of the home.")
                return self.next_search()
            seen = [n for n in CLASSES if self.memory.known(n)
                    and room_of(self.memory.items[n]["x"], self.memory.items[n]["y"]) == room]
            self.say(f"Robot: Done looking around the {room}. "
                     f"I found: {', '.join(seen) or 'nothing I recognise'}. Coming back.")
            self.task = ("home", None, None, None, None)
            return self.set_phase("return")
        if res is None:
            if kind == "fetch":
                self.say(f"Robot: I searched everywhere and couldn't find the {name}. "
                         f"Coming back.")
                self.task = ("home", None, None, None, None)
            else:
                seen = [n for n in CLASSES if self.memory.known(n)]
                self.say(f"Robot: Done exploring in {self.t - self.task_t0:.0f}s. "
                         f"I found: {', '.join(seen) or 'nothing'}. Coming back.")
                self.task = ("home", None, None, None, None)
            return self.set_phase("return")
        self.set_phase("search")
        self.path, self.goal_cell = res

    def goal_mask(self):
        """Where the robot should stand for the current navigation phase."""
        kind, name, _room, _dest, _then = self.task
        P = self.planner
        if self.phase == "goto_room":
            sx, sy = ROOM_SPOTS[self.search_room]
            return P.ring(sx, sy, 0.0, 0.6)
        if self.phase == "search":
            mask = np.zeros_like(P.free)
            mask[self.goal_cell] = True
            return mask
        if self.phase == "approach":
            m = self.memory.known(name)
            return P.ring(m["x"], m["y"], 0.3, self.REACH - 0.07) if m else None
        if self.phase == "deliver" and self.dest[0] == "room":
            sx, sy = ROOM_SPOTS[self.dest[1]]
            return P.ring(sx, sy, 0.0, 0.6)
        if self.phase == "deliver" and self.dest[0] == "furniture":
            pts = drop_points(self.dest[1])               # anywhere the arm can reach the top
            jj, ii = np.mgrid[0:P.ny, 0:P.nx]
            cx, cy = (ii + 0.5) * P.RES, (jj + 0.5) * P.RES
            d = np.min(np.hypot(cx[..., None] - pts[:, 0], cy[..., None] - pts[:, 1]), axis=2)
            return (d >= 0.3) & (d <= self.REACH - 0.07)
        if self.phase == "deliver" and self.dest[0] == "spot":     # back where it was
            return P.ring(self.dest[1], self.dest[2], 0.3, self.REACH - 0.07)
        px, py = PEOPLE[self.dest[1]] if self.phase == "deliver" else PERSON
        return P.ring(px, py, 0.4, 0.75)                     # hand to a person / come back

    def navigate(self, ranges, angles):
        r = self.robot
        kind, name, _room, _dest, _then = self.task
        self.since_plan += 1

        if self.path is None or self.since_plan >= 15:
            mask = self.goal_mask()
            res = None if mask is None else self.planner.plan(r.x, r.y, mask)
            self.since_plan = 0
            if res is None:
                if self.phase == "search":
                    self.blacklist.append(self.planner.centre(*self.goal_cell))
                    self.next_search()
                elif self.phase == "approach":
                    self.say(f"Robot: I can't find a way to the {name} yet, looking around.")
                    self.set_phase("look")
                elif self.phase == "goto_room":
                    self.say(f"Robot: I can't find a way into the {self.search_room}. "
                             f"Searching from here instead.")
                    self.search_room = None
                    if kind == "goto":
                        self.finish(None)
                    else:
                        self.set_phase("look")
                else:
                    self.finish("Robot: I can't find a way back to you.")
                return 0.0, 0.0
            self.path, self.goal_cell = res

        # Close enough?
        gx, gy = self.path[-1]
        arrived = math.hypot(gx - r.x, gy - r.y) < 0.12
        if self.phase == "approach":
            m = self.memory.known(name)
            if m and math.hypot(m["x"] - r.x, m["y"] - r.y) < self.REACH - 0.07:
                arrived = True
        if arrived:
            return self.arrive()

        # Stuck? (bumped into something the map didn't show yet) -> back up, replan
        sx, sy, st = self.stuck_ref
        if self.tick_n - st > 40:
            if math.hypot(r.x - sx, r.y - sy) < 0.1:
                self.resume_phase = self.phase
                self.phase, self.timer = "backup", 8
                return -0.12, 0.6
            self.stuck_ref = (r.x, r.y, self.tick_n)

        # Pure pursuit: aim at a point ~0.35 m ahead along the path
        pts = np.array(self.path)
        k = int(np.argmin(np.hypot(pts[:, 0] - r.x, pts[:, 1] - r.y)))
        target = pts[-1]
        for p in pts[k:]:
            if math.hypot(p[0] - r.x, p[1] - r.y) >= 0.35:
                target = p
                break
        e = wrap(math.atan2(target[1] - r.y, target[0] - r.x) - r.th)
        if abs(e) > 0.7:
            return 0.0, 1.2 * np.sign(e)
        w = float(np.clip(2.0 * e, -1.2, 1.2))
        v = 0.3 * (1.0 - 0.6 * abs(e) / 0.7)
        front = ranges[np.abs(angles) < 0.3].min()
        if front < 0.28:                                   # emergency stop
            v = 0.0
        return v, w

    def arrive(self):
        kind, name, _room, _dest, _then = self.task
        if self.phase == "goto_room":
            if kind == "goto":
                self.finish(f"Robot: I'm in the {self.search_room}.")
            else:
                self.set_phase("look")
        elif self.phase == "search":
            self.set_phase("look")
        elif self.phase == "approach":
            self.set_phase("align")
        elif self.phase == "deliver":
            r = self.robot
            if self.dest[0] == "person":
                self.set_phase("handover", timer=12)
            else:
                if self.dest[0] == "spot":
                    self.drop_point = (self.dest[1], self.dest[2])
                elif self.dest[0] == "furniture":
                    pts = drop_points(self.dest[1])
                    k = int(np.argmin(np.hypot(pts[:, 0] - r.x, pts[:, 1] - r.y)))
                    self.drop_point = tuple(pts[k])
                else:                                       # middle of a room: on the floor
                    self.drop_point = (r.x + 0.35 * math.cos(r.th), r.y + 0.35 * math.sin(r.th))
                self.set_phase("align_drop")
        elif self.phase == "return":
            self.finish("Robot: I'm here.")
        return 0.0, 0.0

    def try_grasp(self, name):
        obj = self.obj(name)
        r = self.robot
        if obj and not obj.carried and math.hypot(obj.x - r.x, obj.y - r.y) <= self.REACH + 0.05:
            # remember where it came from, so "return the stapler" can undo this
            obj.origin = ("person", obj.holder) if obj.holder else ("spot", obj.x, obj.y, obj.host)
            obj.carried = True
            obj.holder = None
            obj.host = None
            self.carrying = obj
            self.memory.forget(name)
            self.say(f"Robot: Got the {name}. Taking {it(name)} {self.dest_text()}.")
            self.set_phase("deliver")
        else:
            self.say(f"Robot: Hmm, the {name} {is_(name)}n't where I thought. Looking again.")
            self.memory.forget(name)
            self.set_phase("look")

    def hand_over(self):
        obj = self.carrying
        self.carrying = None
        obj.carried = False
        person = self.dest[1]
        self.give_to(obj, person)
        if self.task[4] == "back" and person != "you":
            self.say(f"Robot: Here you go, {who(person)}. I'll wait and take the {obj.name} back.")
            self.wait_obj = obj
            return self.set_phase("wait_person", timer=80)
        took = f"(took {self.t - self.task_t0:.0f}s)"
        if person == "you":
            self.finish(f"Robot: Here you go, your {obj.name}! {took}")
        else:
            self.finish(f"Robot: Delivered the {obj.name} to {who(person)} "
                        f"in the {room_of(*PEOPLE[person])}. {took}")

    def give_to(self, obj, person):
        n = sum(o.holder == person for o in self.objects)     # line them up next to them
        px, py = PEOPLE[person]
        obj.place(px - 0.35, py - 0.3 + 0.25 * n, self.segs)
        obj.holder = person
        obj.held_since = self.t
        self.memory.set(obj.name, obj.x, obj.y, self.t)

    def dest_text(self):
        kind, name = self.dest[:2]
        if kind == "spot":
            return f"to {the(self.dest[3])}" if self.dest[3] else "to where I found it"
        if kind == "person":
            return "to you" if name == "you" else f"to {who(name)}"
        if kind == "room":
            return f"to the {name}"
        return f"to {the(name)} ({place_room(self.dest)})"

    def dest_point(self):
        """Where the current task will leave the object (for drawing)."""
        if not self.task or not self.dest or self.task[0] != "fetch":
            return None
        if self.drop_point:
            return self.drop_point
        kind, name = self.dest[:2]
        if kind == "spot":
            return self.dest[1], self.dest[2]
        if kind == "person":
            return PEOPLE[name]
        if kind == "room":
            return ROOM_SPOTS[name]
        x0, y0, x1, y1, _h = FURNITURE[name]
        return ((x0 + x1) / 2, (y0 + y1) / 2)

    def place_object(self):
        obj = self.carrying
        self.carrying = None
        obj.carried = False
        obj.place(*self.drop_point, self.segs)
        self.drop_point = None
        self.memory.set(obj.name, obj.x, obj.y, self.t)
        self.finish(f"Robot: Done. I put the {obj.name} {obj.where()}. "
                    f"(took {self.t - self.task_t0:.0f}s)")

    def put_down(self):
        obj = self.carrying
        self.carrying = None
        obj.carried = False
        r = self.robot
        obj.place(r.x + 0.25 * math.cos(r.th), r.y + 0.25 * math.sin(r.th), self.segs)
        self.memory.set(obj.name, obj.x, obj.y, self.t)
        self.say(f"Robot: I put the {obj.name} down here.")

    @property
    def busy(self):
        return self.task is not None or bool(self.tasks)


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------

FURN_COLORS = {"sofa": "#8e7cc3", "coffee table": "#b08d57", "bed": "#6fa8dc",
               "kitchen table": "#b08d57", "kitchen counter": "#999999", "wardrobe": "#8b6b4a"}
PHASE_TEXT = {"idle": "waiting for a command", "look": "looking around",
              "search": "searching", "goto_room": "driving to the room",
              "approach": "going to the object",
              "align": "lining up", "grasp": "grasping", "deliver": "carrying it over",
              "align_drop": "lining up", "place": "putting it down",
              "wait_person": "waiting for them to finish",
              "handover": "handing it over", "return": "coming back", "backup": "backing up"}


def _hex(c):
    c = c.lstrip("#")
    return np.array([int(c[i:i + 2], 16) / 255 for i in (0, 2, 4)])


class View:
    CAM_W, CAM_H = 192, 108

    def __init__(self, sim, interactive=True):
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Rectangle, Wedge

        self.sim = sim
        self.fig = fig = plt.figure(figsize=(15, 8.4))
        try:
            fig.canvas.manager.set_window_title("Home Robot Starter - fetch demo")
        except Exception:
            pass
        self.ax_w = fig.add_axes([0.01, 0.30, 0.355, 0.60])
        self.ax_m = fig.add_axes([0.375, 0.30, 0.355, 0.60])
        self.ax_c = fig.add_axes([0.745, 0.56, 0.245, 0.34])
        self.ax_k = fig.add_axes([0.745, 0.30, 0.245, 0.24])
        self.ax_l = fig.add_axes([0.01, 0.09, 0.98, 0.17])
        for a in (self.ax_k, self.ax_l):
            a.axis("off")

        # Ground truth
        ax = self.ax_w
        for s in sim.segs:
            ax.plot([s[0], s[2]], [s[1], s[3]], color="#2b2b2b", lw=2)
        for name, (x0, y0, x1, y1, _) in FURNITURE.items():
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, color=FURN_COLORS.get(name, "#b08d57"),
                                   alpha=0.25, lw=0))
            ax.text((x0 + x1) / 2, (y0 + y1) / 2, name, ha="center", va="center",
                    fontsize=6.5, color="#555")
        for name, (x0, y0, x1, y1) in ROOMS:
            ax.text(x0 + 0.12, y1 - 0.15, name.upper(), fontsize=7, color="#999", va="top")
        for person, (px, py) in PEOPLE.items():
            ax.add_patch(Circle((px, py), 0.2, color="#30a46c", zorder=4))
            ax.text(px, py + 0.32, who(person), ha="center", fontsize=8,
                    color="#30a46c", weight="bold")
        self.fov = Wedge((0, 0), Camera.RANGE, 0, 1, color="#ffd60a", alpha=0.18, lw=0)
        ax.add_patch(self.fov)
        self.obj_w = []
        for o in sim.objects:
            p = Rectangle((0, 0), 0.16, 0.16, color=o.color, zorder=6)
            t = ax.text(0, 0, o.name, fontsize=7, color=o.color, zorder=6, weight="bold")
            ax.add_patch(p)
            self.obj_w.append((o, p, t))
        self.trail, = ax.plot([], [], color="#d9822b", lw=0.8, alpha=0.6)
        self.body = Circle((0, 0), sim.robot.RADIUS, color="#2f6fdb", zorder=7)
        ax.add_patch(self.body)
        self.head, = ax.plot([], [], color="white", lw=2, zorder=8)
        ax.set_title("Real world (what Gazebo simulates)", fontsize=10)

        # Robot's map
        ax = self.ax_m
        self.map_img = ax.imshow(np.zeros((sim.grid.ny, sim.grid.nx, 3)), origin="lower",
                                 extent=[0, WORLD_W, 0, WORLD_H], interpolation="nearest")
        self.path_line, = ax.plot([], [], color="#30a46c", lw=2)
        self.goal_dot, = ax.plot([], [], "*", color="#30a46c", ms=13)
        self.dest_marks = [a.plot([], [], "X", color="#f76b15", ms=12, mec="white", mew=1.2,
                                  zorder=9)[0] for a in (self.ax_w, ax)]
        self.dot_m = Circle((0, 0), sim.robot.RADIUS, color="#2f6fdb", zorder=7)
        ax.add_patch(self.dot_m)
        self.head_m, = ax.plot([], [], color="white", lw=2, zorder=8)
        for person, (px, py) in PEOPLE.items():
            ax.plot(px, py, "o", color="#30a46c", ms=8)
            ax.text(px, py - 0.4, who(person), ha="center", fontsize=7, color="#30a46c")
        for name, (x0, y0, x1, y1) in ROOMS:          # the room labels stored with the map
            ax.text(x0 + 0.12, y1 - 0.15, name.upper(), fontsize=7, color="#888", va="top",
                    zorder=3)
        self.room_box = Rectangle((0, 0), 1, 1, fill=False, ec="#f76b15", lw=2.5, ls="--",
                                  visible=False, zorder=5)
        ax.add_patch(self.room_box)
        self.mem_marks = {}
        ax.set_title("Robot's map: LiDAR walls + objects it has recognised\n"
                     "(blue tint = floor the camera has already checked)", fontsize=10)

        for a in (self.ax_w, self.ax_m):
            a.set_xlim(-0.2, WORLD_W + 0.2)
            a.set_ylim(-0.2, WORLD_H + 0.2)
            a.set_aspect("equal")
            a.set_xticks([]); a.set_yticks([])

        # Camera
        ax = self.ax_c
        self.cam_img = ax.imshow(np.zeros((self.CAM_H, self.CAM_W, 3)),
                                 extent=[0, self.CAM_W, self.CAM_H, 0], interpolation="nearest")
        ax.set_xlim(0, self.CAM_W); ax.set_ylim(self.CAM_H, 0)
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_title("Robot camera + object detector", fontsize=10)
        self.cam_obj = {}
        for o in sim.objects:
            body = Rectangle((0, 0), 1, 1, color=o.color, visible=False)
            bb = Rectangle((0, 0), 1, 1, fill=False, ec="#00e676", lw=1.5, visible=False)
            lab = ax.text(0, 0, "", fontsize=7, color="black", visible=False,
                          bbox=dict(boxstyle="square,pad=0.1", fc="#00e676", lw=0))
            lab.set_clip_on(True)                       # keep labels inside the camera view
            ax.add_patch(body); ax.add_patch(bb)
            self.cam_obj[o.name] = (body, bb, lab)

        self.know = self.ax_k.text(0, 1, "", va="top", family="monospace", fontsize=8.5)
        self.log = self.ax_l.text(0, 1, "", va="top", family="monospace", fontsize=9)
        self.status = fig.text(0.01, 0.275, "", family="monospace", fontsize=9.5, weight="bold")
        fig.text(0.5, 0.965, f"Tell the robot what to do. Type in the box below (e.g. {EXAMPLES})",
                 ha="center", fontsize=10.5)
        fig.text(0.5, 0.935, f"keys: 1-6 fetch {'/'.join(o[0] for o in OBJECTS[:6])}   e explore   "
                 "h come here   c cancel   +/- speed   r reset   q quit",
                 ha="center", fontsize=8.5, color="#666")
        self.speed = 2
        self.textbox = None
        if interactive:
            from matplotlib.widgets import TextBox
            tax = fig.add_axes([0.16, 0.02, 0.6, 0.045])
            self.textbox = TextBox(tax, "Say to robot:  ", initial="")
            self.textbox.on_submit(self.on_submit)
            fig.canvas.mpl_connect("key_press_event", self.on_key)

    # --- input -------------------------------------------------------------

    def on_submit(self, text):
        if text.strip():
            self.sim.command(text)
            self.textbox.set_val("")

    def on_key(self, ev):
        import matplotlib.pyplot as plt
        if self.textbox is not None and self.textbox.capturekeystrokes:
            return
        names = [o[0] for o in OBJECTS]
        if ev.key in [str(i + 1) for i in range(len(names))]:
            self.sim.command(f"bring me the {names[int(ev.key) - 1]}")
        elif ev.key == "e":
            self.sim.command("explore the house")
        elif ev.key == "h":
            self.sim.command("come here")
        elif ev.key == "c":
            self.sim.command("stop")
        elif ev.key in ("+", "="):
            self.speed = min(self.speed * 2, 16)
        elif ev.key == "-":
            self.speed = max(self.speed // 2, 1)
        elif ev.key == "r":
            self.sim.reset()
        elif ev.key == "q":
            plt.close(self.fig)

    # --- camera image (a tiny raycaster) -------------------------------------

    def render_camera(self):
        sim, r = self.sim, self.sim.robot
        W, H, ch = self.CAM_W, self.CAM_H, Camera.HEIGHT
        f = (W / 2) / math.tan(Camera.FOV / 2)
        rel = np.linspace(Camera.FOV / 2, -Camera.FOV / 2, W)
        rows = np.arange(H)[:, None] + 0.5
        img = np.empty((H, W, 3))
        img[:H // 2] = [0.93, 0.93, 0.95]
        img[H // 2:] = np.linspace(0.78, 0.62, H - H // 2)[:, None, None] * [1.0, 0.95, 0.88]

        if not hasattr(self, "_walls"):
            heights = np.full(len(sim.segs), WALL_H)
            colors = np.tile([0.85, 0.82, 0.76], (len(sim.segs), 1))
            for name, (x0, y0, x1, y1, h) in FURNITURE.items():
                for s in box(x0, y0, x1, y1):
                    k = np.where(np.all(np.isclose(sim.segs, s), axis=1))[0]
                    heights[k] = h
                    colors[k] = _hex(FURN_COLORS.get(name, "#b08d57"))
            self._seg_h, self._seg_c = heights, colors
            self._walls = sim.segs[heights >= WALL_H]

        d_wall, _ = raycast(r.x, r.y, r.th + rel, self._walls)
        perp = np.maximum(d_wall * np.cos(rel), 0.05)
        top = H / 2 - f * (WALL_H - ch) / perp
        bot = H / 2 + f * ch / perp
        m = (rows >= top) & (rows <= bot)
        shade = np.clip(1.05 - perp / 9, 0.45, 1.0)
        img[m] = (np.array([0.85, 0.82, 0.76])[None, :] * shade[:, None])[np.nonzero(m)[1]]

        d_all, idx = raycast(r.x, r.y, r.th + rel, sim.segs)
        fh = self._seg_h[idx]
        is_f = (fh < WALL_H) & (d_all < d_wall - 1e-6)
        perp_f = np.maximum(d_all * np.cos(rel), 0.05)
        ftop = H / 2 - f * (fh - ch) / perp_f
        fbot = H / 2 + f * ch / perp_f
        mf = is_f[None, :] & (rows >= ftop) & (rows <= fbot)
        sf = np.clip(1.05 - perp_f / 9, 0.45, 1.0)
        img[mf] = (self._seg_c[idx] * sf[:, None])[np.nonzero(mf)[1]]
        self.cam_img.set_data(np.clip(img, 0, 1))

        detected = {d[0]: d[1] for d in sim.detections}
        for o in sim.objects:
            body, bb, lab = self.cam_obj[o.name]
            vis = sim.camera.visible(r, o)
            if vis is None:
                for a in (body, bb, lab):
                    a.set_visible(False)
                continue
            d, b = vis
            pd = max(d * math.cos(b), 0.05)
            cx = W / 2 - f * math.tan(b)
            w = max(f * o.size[0] / pd, 2)
            h = max(f * o.size[1] / pd, 2)
            ybot = H / 2 + f * (ch - o.z) / pd
            body.set_bounds(cx - w / 2, ybot - h, w, h)
            body.set_visible(True)
            if o.name in detected:
                pad = 2
                bb.set_bounds(cx - w / 2 - pad, ybot - h - pad, w + 2 * pad, h + 2 * pad)
                lab.set_position((cx - w / 2 - pad, ybot - h - pad - 2))
                lab.set_text(f"{o.name} {detected[o.name]:.2f}")
                bb.set_visible(True); lab.set_visible(True)
            else:
                bb.set_visible(False); lab.set_visible(False)

    # --- frame -------------------------------------------------------------

    def update(self, _frame=None, steps=None):
        sim = self.sim
        for _ in range(steps or self.speed):
            sim.tick()
        r = sim.robot
        self.body.center = (r.x, r.y)
        self.dot_m.center = (r.x, r.y)
        hx, hy = [r.x, r.x + 0.24 * math.cos(r.th)], [r.y, r.y + 0.24 * math.sin(r.th)]
        self.head.set_data(hx, hy)
        self.head_m.set_data(hx, hy)
        self.fov.set_center((r.x, r.y))
        deg = math.degrees(r.th)
        half = math.degrees(Camera.FOV / 2)
        self.fov.set_theta1(deg - half); self.fov.set_theta2(deg + half)
        if not hasattr(self, "_trail"):
            self._trail = []
        self._trail.append((r.x, r.y))
        self._trail = self._trail[-4000:]
        tr = np.array(self._trail)
        self.trail.set_data(tr[:, 0], tr[:, 1])
        for o, p, t in self.obj_w:
            p.set_xy((o.x - 0.08, o.y - 0.08))
            t.set_position((o.x + 0.12, o.y + 0.05))

        # map with camera coverage tint
        g = sim.grid.image()
        rgb = np.repeat(g[:, :, None], 3, axis=2)
        viewed = np.kron(sim.planner.viewed, np.ones((2, 2), dtype=bool))
        tint = viewed & (g > 0.8)
        rgb[tint] = [0.80, 0.90, 1.0]
        self.map_img.set_data(rgb)
        if sim.path:
            p = np.array(sim.path)
            self.path_line.set_data(p[:, 0], p[:, 1])
            self.goal_dot.set_data([p[-1, 0]], [p[-1, 1]])
        else:
            self.path_line.set_data([], [])
            self.goal_dot.set_data([], [])

        dp = sim.dest_point()
        for mk in self.dest_marks:
            mk.set_data(*([[dp[0]], [dp[1]]] if dp else [[], []]))
        if sim.search_room:
            x0, y0, x1, y1 = ROOM_RECTS[sim.search_room]
            self.room_box.set_bounds(x0 + 0.05, y0 + 0.05, x1 - x0 - 0.1, y1 - y0 - 0.1)
            self.room_box.set_visible(True)
        else:
            self.room_box.set_visible(False)

        # recognised objects on the robot's map
        for name in list(self.mem_marks):
            if not sim.memory.known(name):
                for a in self.mem_marks.pop(name):
                    a.remove()
        for name in CLASSES:
            m = sim.memory.known(name)
            if not m:
                continue
            o = sim.obj(name)
            col = o.color if o else "k"
            if name not in self.mem_marks:
                mk, = self.ax_m.plot([], [], "s", color=col, ms=7, mec="k", mew=0.8, zorder=6)
                tx = self.ax_m.text(0, 0, "", fontsize=7, color=col, weight="bold", zorder=6)
                self.mem_marks[name] = (mk, tx)
            mk, tx = self.mem_marks[name]
            mk.set_data([m["x"]], [m["y"]])
            tx.set_position((m["x"] + 0.12, m["y"] + 0.08))
            tx.set_text(f"{name} {100 * m['score']:.0f}%")

        self.render_camera()

        lines = ["WHAT THE ROBOT KNOWS", ""]
        for name in CLASSES:
            m = sim.memory.known(name)
            o = sim.obj(name)
            if o is not None and o.carried:
                s = "in my gripper"
            elif m:
                place = f"with {who(o.holder)}" if (o and o.holder) else \
                    (furniture_at(m["x"], m["y"]) or "floor") + ", " + room_of(m["x"], m["y"])
                s = f"{place} {100 * m['score']:.0f}%"
            else:
                s = "not seen yet"
            lines.append(f"{name:8} {s}")
        self.know.set_text("\n".join(lines))

        self.log.set_text("\n".join(f"{t:6.1f}s  {m}" for t, m in sim.messages[-7:]))
        task = sim.task
        what = "-"
        if task:
            what = " ".join(str(p) for p in task[:2] if p)
            if task[0] == "fetch" and task[3] and task[3] != ("person", "you"):
                d = task[3]
                what += " -> " + (who(d[1]) if d[0] == "person" else
                                  (d[3] or "its spot") if d[0] == "spot" else d[1])
                if task[4] == "back":
                    what += " & back"
        queued = f"  (+{len(sim.tasks)} queued)" if sim.tasks else ""
        self.status.set_text(
            f"t={sim.t:6.1f}s  task: {what:22} doing: {PHASE_TEXT.get(sim.phase, sim.phase):22}"
            f"/cmd_vel v={sim.cmd[0]:+.2f} w={sim.cmd[1]:+.2f}   speed x{self.speed}{queued}")


# ---------------------------------------------------------------------------
# Entry points
# ---------------------------------------------------------------------------

def run_interactive(commands):
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    for k in list(matplotlib.rcParams):
        if k.startswith("keymap."):
            matplotlib.rcParams[k] = []
    sim = FetchSim()
    view = View(sim, interactive=True)
    sim.say("Robot: Hi! I'm ready. Tell me what to bring you.")
    for c in commands:
        sim.command(c)
    anim = FuncAnimation(view.fig, view.update, interval=30, cache_frame_data=False)
    plt.show()
    return anim


def run_recorded(commands, gif, png, max_seconds, steps_per_frame):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sim = FetchSim()
    view = View(sim, interactive=False)
    for c in commands:
        sim.command(c)
    max_ticks = int(max_seconds / FetchSim.DT)

    def done():
        return (not sim.busy) or sim.tick_n >= max_ticks

    if gif:
        from matplotlib.animation import FuncAnimation, PillowWriter

        def frames():
            n = 0
            while not done():
                yield n
                n += 1
            for _ in range(20):                        # hold the last frame ~1.5 s
                yield -1

        def step(n):
            view.update(steps=steps_per_frame if n >= 0 else 0)

        print(f"Recording {gif} ...")
        anim = FuncAnimation(view.fig, step, frames=frames, cache_frame_data=False,
                             save_count=100000)
        anim.save(gif, writer=PillowWriter(fps=12), dpi=70)
        print(f"Saved {gif}")
    else:
        while not done():
            sim.tick()
        view.update(steps=0)
        view.fig.savefig(png, dpi=100)
        print(f"Saved {png}")
    if sim.busy:
        print(f"Stopped after {max_seconds}s of simulated time (task still running).")
    print(f"Simulated {sim.t:.0f}s, drove {sim.robot.odom_dist:.1f} m, bumps={sim.robot.bumps}")
    plt.close(view.fig)
    return sim


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", choices=sorted(WORLDS), default="office",
                    help="office (default) or home")
    ap.add_argument("--say", action="append", default=[], metavar="TEXT",
                    help='a command for the robot, e.g. --say "bring me the cup" (repeatable)')
    ap.add_argument("--gif", nargs="?", const="fetch.gif", default=None,
                    help="record the --say commands to a GIF instead of opening a window")
    ap.add_argument("--headless", action="store_true",
                    help="no window: run the --say commands and save fetch_result.png")
    ap.add_argument("--out", default="fetch_result.png", help="image for --headless")
    ap.add_argument("--seconds", type=float, default=900, help="max simulated time (recordings)")
    ap.add_argument("--frame-steps", type=int, default=6,
                    help="simulation steps per GIF frame (higher = shorter GIF)")
    args = ap.parse_args()
    use_world(args.world)

    if args.gif or args.headless:
        cmds = args.say or DEFAULT_SAY
        run_recorded(cmds, args.gif, args.out, args.seconds, args.frame_steps)
    else:
        run_interactive(args.say)


if __name__ == "__main__":
    sys.exit(main())
