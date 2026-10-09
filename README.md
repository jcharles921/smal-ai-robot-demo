# Home Robot Starter

A small, working starting point for home and office robotics with **ROS 2 Jazzy + Gazebo + linorobot2**, in three parts:

| Part | What it is | Needs | Time to first run |
|---|---|---|---|
| **A. `demo/sim_demo.py`** | A 2D robot that explores an apartment with a simulated LiDAR and builds a map live | Python 3 only | ~1 minute |
| **A2. `demo/fetch_demo.py`** | An **office delivery robot**: you type what you want in plain English ("take this to Diana", "bring the contract back"), and it finds objects with its camera, picks them up, delivers them to people, desks or rooms, and brings them back | Python 3 only | ~1 minute |
| **B. `ros2_ws/` + `scripts/`** | The real thing: linorobot2 in Gazebo, SLAM Toolbox, Nav2, plus your own ROS 2 node (`home_explorer`) that drives the robot autonomously | Ubuntu 24.04 (native or WSL2) | ~30–60 min (mostly downloads) |

Part A uses **the same exploration algorithm** as the ROS 2 node in part B, so what you see in the demo is what the robot does in Gazebo.

```
smal-ai-robot-demo/
├── demo/
│   ├── sim_demo.py          ← run this now: exploration + mapping
│   ├── fetch_demo.py        ← office delivery robot (find, bring, bring back)
│   ├── requirements.txt     numpy + matplotlib
│   ├── run_demo.bat         ← Windows: double-click
│   └── run_demo.sh          ← Linux / macOS / WSL
├── ros2_ws/src/home_explorer/   ← your ROS 2 package
│   ├── home_explorer/controller.py     exploration logic (pure Python, unit-tested)
│   ├── home_explorer/explorer_node.py  /scan → /cmd_vel node
│   ├── config/explorer.yaml            tuning
│   ├── launch/explore.launch.py
│   └── test/test_controller.py
└── scripts/
    ├── 1_install_ros2_jazzy.sh   ROS 2 Jazzy + Gazebo Harmonic + Nav2 + SLAM Toolbox
    ├── 2_setup_workspace.sh      clones linorobot2, rosdep, colcon build
    └── run.sh                    gazebo | check | slam | explore | teleop | savemap | nav
```

---

## A. Run the demo (Python only, about 1 minute)

**What you'll see:** a robot (the blue dot) drives itself around an apartment it has never seen. It uses a laser scanner (the red lines) to avoid walls and furniture, and as it moves it draws its own map of the place (the right panel), filling in the rooms it has explored.

### 1. Get the code

```bash
git clone https://gitlab.dccsintra.net/sandbox/cmwunguz/ai_mini_robot.git
cd ai_mini_robot
```

No git? On the GitHub page, click **Code → Download ZIP** and unzip it.

### 2. Install Python and the two libraries

The demo needs **Python 3.9 or newer** (check with `python3 --version`, or `python --version` on Windows) and two libraries, `numpy` and `matplotlib`, which are listed in `demo/requirements.txt`.

- **Windows:** install Python from [python.org](https://www.python.org/downloads/) and tick **Add python.exe to PATH** in the installer. That's all; `run_demo.bat` installs the libraries for you.
- **macOS:** install Python from python.org, or with Homebrew (`brew install python`). Then, from the project folder, create a virtual environment once:
  ```bash
  python3 -m venv .venv
  source .venv/bin/activate
  pip install -r demo/requirements.txt
  ```
  Run `source .venv/bin/activate` again in each new terminal, then use the `python demo/...` commands below. (`run_demo.sh` also works: it installs the libraries for your user instead.)
- **Ubuntu / Debian / WSL:**
  ```bash
  sudo apt install python3 python3-pip python3-venv python3-tk
  ```
  `python3-tk` is what lets matplotlib open a window. Ubuntu 24.04 refuses system-wide `pip install`, so put the libraries in a virtual environment:
  ```bash
  python3 -m venv .venv
  source .venv/bin/activate
  pip install -r demo/requirements.txt
  ```
  Run `source .venv/bin/activate` again in each new terminal before starting the demo.

### 3. Run it

| Where | Command |
|---|---|
| Windows | double-click `demo\run_demo.bat`, or run `python demo\sim_demo.py` |
| macOS / Linux / WSL | `bash demo/run_demo.sh` |
| Inside a virtual environment | `python demo/sim_demo.py` |
| GitHub Codespaces, SSH, or a server | the same command; see below |

With a screen, a window opens with two panels. The left is the "real" world (like Gazebo) with the LiDAR beams in red. The right is the map the robot builds from those beams (like the `/map` that SLAM Toolbox publishes). The status bar shows the `/cmd_vel` it is sending. Click inside the window, then use:

| Key | Action |
|---|---|
| `m` | toggle AUTO / MANUAL |
| arrow keys | drive in MANUAL (↑↓ speed, ←→ turn) |
| space | stop |
| `r` | reset robot and map |
| `p` | save the map to `map.png` |
| `q` | quit |

**No screen (Codespaces, SSH, a server):** the demo detects this and records `demo.gif` instead. That's a 20-second clip of 90 simulated seconds, and it takes about a minute to render. In VS Code or Codespaces, right-click the file in the explorer and choose **Open Preview**.

### Other options

| Command | What it does |
|---|---|
| `python3 demo/sim_demo.py --gif` | record `demo.gif` even when you have a screen |
| `python3 demo/sim_demo.py --gif run.gif --seconds 180` | longer recording with your own file name |
| `python3 demo/sim_demo.py --headless` | no animation: simulates about 4 minutes, then saves one picture, `demo_result.png`, and prints how much was mapped |
| `python3 demo/sim_demo.py -h` | list all options |

Output files (`demo.gif`, `demo_result.png`, `map.png`) are saved in the folder you run the command from. With `run_demo.sh` or `run_demo.bat`, that's `demo/`. In testing, the robot maps about 80% of the apartment in 5 simulated minutes without hitting anything.

### Troubleshooting

| Problem | Fix |
|---|---|
| No window opens and you see `UserWarning: Animation was deleted without rendering anything` | On Ubuntu with a screen, install `python3-tk` (`sudo apt install python3-tk`). On a machine with no screen, add `--gif`. |
| `error: externally-managed-environment` when installing with pip | Use the virtual environment steps in step 2. |
| `python3` not found on Windows | Use `python` or `py` instead, or reinstall Python with **Add python.exe to PATH** ticked. |
| The keys do nothing | Click inside the window first so it has keyboard focus. |

**Things to try:** edit `build_world()` in `demo/sim_demo.py` to draw your own home, or change `ExplorerController`'s `stop_dist` / `max_speed` and watch how the behaviour changes.

---

## A2. The office delivery robot (`demo/fetch_demo.py`)

The same robot, now with a **camera and object detector**, a **memory** of where things are and who has them, a **path planner** and a **gripper**. You give it jobs in plain English, and it searches the office, picks things up, delivers them, and brings them back when you ask.

```bash
python demo/fetch_demo.py                  # the office (default)
python demo/fetch_demo.py --world home     # the apartment, with Mom and Dad
```

Type a command in the box at the bottom of the window and press **Enter**. You can type the next command while the robot is busy: commands are queued and done in order.

### The office

```
 y=9 +---------------+-----------+------------------------+
     |  MEETING ROOM |  MANAGER  |        PANTRY          |
     |  laptop       |  Diana    |  mug                   |
 5.4 +-----door------+---door----+------door--------------+
     |                    CORRIDOR                         |
 4.0 +----door--------door------+-door-+---door----------+
     |  OPEN OFFICE             | PRINT|   RECEPTION     |
     |  you, Alice, Bob, Carol  | room |   Eve, parcel   |
 y=0 +--------------------------+------+-----------------+
```

| | |
|---|---|
| **People** | you (at your desk), Alice, Bob, Carol (open office), Diana, the manager (her office), Eve, the receptionist |
| **Objects** | stapler (Bob's desk), laptop (meeting table), printout (printer), mug (pantry table), keys (reception desk), charger (Carol's desk), folder (manager's desk), parcel (reception floor), contract (**you are holding it**) |
| **Not in the office** | scissors: the detector knows them, so asking for them shows what happens when something can't be found |

The left panel is the real office, with the camera's view as a yellow cone. The middle panel is the robot's own map: walls from the LiDAR, objects it has recognised (with confidence), its planned route (green), its destination (orange **X**), and in blue the floor its camera has already checked. On the right are the live camera image with detection boxes and the list of where every object is.

### Sending things and bringing them back

Delivering and bringing back are **separate commands**. Send something now, and ask for it back later only if you want it:

| You type | What the robot does |
|---|---|
| `take this to Diana` | comes to you, takes what you're holding (the contract), gives it to Diana. Done. |
| `bring the contract back` / `get my contract back from Diana` | later, whenever you want: goes to whoever has it now, collects it, brings it back to you |
| `bring me the stapler` | finds the stapler and hands it to you |
| `return the stapler` / `put it back` | takes it back exactly where it came from (Bob's desk), or to the person it came from |
| `take my laptop to Alice`, later `bring my laptop back` | lends it to Alice, then collects it from her |
| `take the folder to Diana and bring it back` | optional, both in one command: hands it over, waits while she uses it, brings it back |

"Bring it back" / "put it back" mean the **last object you mentioned**, so naming the object ("bring the **contract** back") always works.

### Where it can deliver

| Destination | Examples | What it does |
|---|---|---|
| **a person** | `give the report to the manager`, `send the keys to Eve`, `bring Bob the charger` | drives to them and hands it over. "The manager" and "the boss" mean Diana, "the receptionist" means Eve. |
| **a desk or other furniture** | `put the printout on Bob's desk`, `leave the mug on my desk` | drives to where its arm can reach the top and places it just inside the edge |
| **a room** | `send the parcel to reception`, `take the mug to the pantry` | puts it on the floor near the room's labelled spot |

If you don't name a destination, it brings things to you. It refuses what it can't do and says why: someone it doesn't know ("give the keys to Frank"), a place that isn't on its map ("the garage"), or a surface too high for its arm (`put the mug on the supply shelf`). In those cases it brings the object to you instead.

### Finding things

| You type | What the robot does |
|---|---|
| `bring me the laptop` | if it has seen the laptop before, goes straight there; if not, searches: drives to the spot with the most unchecked floor, turns around once with its camera, repeats until it finds it |
| `bring me the laptop from the meeting room` | goes to the meeting room first and searches only there (outlined in orange). If it isn't there, says so and searches everywhere else. |
| `the keys are on the reception desk, bring them to me` | naming a piece of furniture counts as naming its room |
| `where is the contract?` | answers from memory: "Diana has the contract", "The stapler is on Bob's desk in the open office (91% sure, seen 40s ago)" |
| `find the scissors` | searches the whole office, then reports it couldn't find them |
| `explore the office` / `explore the pantry` | looks around everywhere (or one room) and remembers every object it sees |
| `go to the print room` / `come here` / `stop` | drives there / comes back to you / cancels everything (putting down what it holds) |

**Keys** (when you're not typing in the box): `1`–`6` fetch the stapler, laptop, printout, mug, keys and charger, `e` explore, `h` come here, `c` cancel, `+` / `-` change the simulation speed, `r` reset, `q` quit.

### Without a window

| Command | What it does |
|---|---|
| `python demo/fetch_demo.py --say "take this to Diana"` | opens the window and starts with that command (`--say` can be repeated) |
| `python demo/fetch_demo.py --gif fetch.gif` | records a GIF of the default script: "take this to Diana", "bring me the stapler", "bring the contract back" |
| `python demo/fetch_demo.py --gif fetch.gif --say "bring me the laptop"` | records your own commands |
| `python demo/fetch_demo.py --headless --say "bring me the mug"` | no animation: runs the commands, prints the robot's messages, saves `fetch_result.png` |
| `python demo/fetch_demo.py -h` | list all options |

In testing, every delivery reached the right person or place with zero collisions. For example, "take this to Diana" then "bring the contract back" took 37 s and 48 s of simulated time, and "bring me the stapler" took 24 s.

### Make it your office

Everything about the office is in the `OFFICE` settings near the top of `demo/fetch_demo.py`:

| Setting | What it is |
|---|---|
| `WALLS`, `FURNITURE` | the floor plan: wall segments, and each desk or table as a rectangle with its height |
| `ROOMS`, `ROOM_SPOTS`, `ROOM_WORDS` | each room's area, a spot the robot drives to, and the names people use for it |
| `PEOPLE`, `PEOPLE_WORDS` | your colleagues, where they sit, and what people call them ("manager", "boss") |
| `OBJECTS`, `CLASSES`, `HELD_AT_START` | what's lying around, everything the detector can recognise, and what someone is holding at the start |

The original home layout is the `--world home` version, with Mom and Dad and the same commands.

### How it maps to a real robot

| In the demo | On the real robot |
|---|---|
| `Camera.detect()`: an 80° camera that only sees unoccluded objects within 3.5 m, with noisy scores | a YOLO node on the camera image publishing `vision_msgs/Detection2DArray`, plus depth to get each object's position |
| `Memory`: an object counts as found after 3 detections; it also tracks who holds what and where each thing came from | a semantic map node that stores labelled poses in the `map` frame |
| `ROOMS`, `ROOM_SPOTS`, `FURNITURE` | named areas and points you mark once on the saved SLAM map (for example by clicking in RViz) and store in a YAML file |
| `Planner`: a costmap from the LiDAR map, plus Dijkstra path planning | Nav2 (`nav2_simple_commander`, `BasicNavigator.goToPose`) |
| search: viewpoints with the most unchecked floor, turn around, repeat | frontier or viewpoint exploration that sends Nav2 goals |
| `align` → `grasp` → `deliver` → `handover` / `place` | MoveIt 2 for the arm, Nav2 goals to the person, desk or room |
| `PEOPLE` at fixed seats | a person detector or face recognition, checking each person's desk first |

For a real office, also plan for: the person **confirming** each hand-over (a button on the robot, or a reply on their phone or Slack), a **lockable compartment** for things like contracts and laptops, a log of every hand-off, and access to doors and lifts.

---

## B. The full ROS 2 + Gazebo + linorobot2 setup

linorobot2's current branch is **`jazzy`**, which means **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**.

### 0. Windows only: get Ubuntu 24.04 via WSL2
In PowerShell (as administrator, once):
```powershell
wsl --install -d Ubuntu-24.04
```
Reboot if asked, open "Ubuntu 24.04" from the Start menu and create a user. Windows 11 shows Linux GUI apps (Gazebo, RViz) automatically.

Clone the project **inside the Linux side**, not under `/mnt/c`, where builds are very slow:
```bash
git clone https://github.com/jcharles921/smal-ai-robot-demo.git ~/smal-ai-robot-demo
cd ~/smal-ai-robot-demo
```

### 1. Install ROS 2 Jazzy (one time, ~10–30 min)
```bash
bash scripts/1_install_ros2_jazzy.sh
```

### 2. Build the workspace (one time, ~5 min)
```bash
bash scripts/2_setup_workspace.sh          # or: ... 4wd / mecanum
source ~/.bashrc
```
This clones `linorobot2` (branch `jazzy`) and `linorobot2_viz` into `ros2_ws/src/` next to `home_explorer`, installs dependencies with rosdep (skipping the micro-ROS keys, as linorobot2's docs recommend), builds, and sets `LINOROBOT2_BASE`.

### 3. Run it: one command per terminal

```bash
# Terminal 1 - Gazebo with the robot
bash scripts/run.sh gazebo                 # or: bash scripts/run.sh gazebo playground

# Terminal 2 - sanity check before SLAM: /scan rate, topics, odom→base_footprint TF
bash scripts/run.sh check

# Terminal 2 - SLAM Toolbox + RViz: the map appears as the robot moves
bash scripts/run.sh slam

# Terminal 3 - your node drives the robot around by itself
bash scripts/run.sh explore
#   pause:  ros2 param set /explorer enabled false
#   or drive yourself instead:  bash scripts/run.sh teleop

# When the map looks complete
bash scripts/run.sh savemap my_home
```

Then navigate on the saved map (stop SLAM and the explorer first):
```bash
bash scripts/run.sh nav ros2_ws/maps/my_home.yaml
```
In RViz click **2D Pose Estimate** where the robot is, then **2D Goal Pose** somewhere else, and Nav2 drives there.

**Gazebo window black or blank on WSL2?** Add `--soft` at the end: `bash scripts/run.sh gazebo --soft` (software rendering: slower, but works).

### How the pieces talk

```
Gazebo (linorobot2_gazebo)
   │  /scan  (LaserScan, via ros_gz_bridge)
   │  /odom, /imu, /tf
   ▼
home_explorer/explorer_node ──/cmd_vel (Twist)──► Gazebo diff-drive plugin
   │
SLAM Toolbox (linorobot2_navigation slam.launch.py)
   reads /scan + TF ──► publishes /map and map→odom TF ──► RViz
```

`explorer_node` only uses `/scan` and `/cmd_vel`, the same interface a physical linorobot2 exposes, so it runs on real hardware too with `ros2 launch home_explorer explore.launch.py sim:=false`. Test in simulation first, and keep a hand near the power switch.

### Tuning (`ros2_ws/src/home_explorer/config/explorer.yaml`)
| Param | Default | Meaning |
|---|---|---|
| `max_speed` | 0.25 m/s | top forward speed |
| `stop_dist` | 0.6 m | obstacle ahead closer than this → turn in place |
| `side_dist` | 0.32 m | obstacle at the side closer than this → turn away |
| `max_runtime_s` | 0 | stop after N seconds (0 = never) |

Change params live: `ros2 param set /explorer max_speed 0.15`. Because of `--symlink-install`, edits to Python files take effect the next time you launch, no rebuild needed (rebuild after adding new files).

Run the unit tests (no ROS needed): `cd ros2_ws/src/home_explorer && python3 -m pytest test/`

---

## Next steps
1. **Your own world:** `ros2 run linorobot2_gazebo image_to_gazebo` turns a floor-plan image of your home into a Gazebo world, then `bash scripts/run.sh gazebo <world_name>`.
2. **Smarter exploration:** subscribe to `/map` in `explorer_node` and steer toward frontiers (edges between known-free and unknown cells) instead of random curiosity.
3. **Send Nav2 goals from code:** use `nav2_simple_commander` (`BasicNavigator.goToPose`) to build a "patrol the rooms" node.
4. **Office delivery on the real robot:** port `demo/fetch_demo.py`'s task logic into a ROS 2 node: a detector node (YOLO) for objects, Nav2 for driving to `ROOM_SPOTS` / people / desks, and MoveIt 2 for the arm.
5. **Real hardware:** follow [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware); the same SLAM/Nav2 launch files work without `sim:=true`.

## References
- linorobot2: https://github.com/linorobot/linorobot2 · docs: https://linorobot.github.io/linorobot2/
- ROS 2 Jazzy install: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Nav2 setup guides: https://docs.nav2.org/setup_guides/index.html