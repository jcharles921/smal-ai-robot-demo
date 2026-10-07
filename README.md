# Home Robot Starter

A small, working starting point for home robotics with **ROS 2 Jazzy + Gazebo + linorobot2**, in two parts:

| Part | What it is | Needs | Time to first run |
|---|---|---|---|
| **A. `demo/`** | A 2D robot that explores an apartment with a simulated LiDAR and builds a map live | Python 3 only | ~1 minute |
| **B. `ros2_ws/` + `scripts/`** | The real thing: linorobot2 in Gazebo, SLAM Toolbox, Nav2, plus your own ROS 2 node (`home_explorer`) that drives the robot autonomously | Ubuntu 24.04 (native or WSL2) | ~30–60 min (mostly downloads) |

Part A uses **the same exploration algorithm** as the ROS 2 node in part B, so what you see in the demo is what the robot does in Gazebo.

```
smal-ai-robot-demo/
├── demo/
│   ├── sim_demo.py          ← run this now
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
- **macOS:** install Python from python.org, or with Homebrew (`brew install python`). `run_demo.sh` installs the libraries for you.
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

### Fetch demo: "find it, bring it, bring it back"

The demo opens in an **office** by default. It has an open-plan area with desks (yours, Alice's, Bob's, Carol's), a corridor, a meeting room, the manager's office (Diana), a pantry, a print room and reception (Eve). Things you can type:

| Command | What the robot does |
|---|---|
| `bring me the stapler` | finds it and hands it to you |
| `take this to Diana` | takes what you're holding (the contract) and gives it to Diana, then it's done |
| `bring the contract back` / `get my contract back from Diana` | later, whenever you want: goes to whoever has it now, collects it, brings it back to you |
| `take the folder to Diana and bring it back` | optional, both in one command: hands it over, waits while she's busy with it, brings it back |
| `return the stapler` / `put it back` | takes it back to exactly where it came from (Bob's desk, or the person it came from) |
| `put the printout on Bob's desk` / `send the parcel to reception` | delivers to a desk, a piece of furniture or a room |
| `give the report to the manager` | "the manager" and "the boss" mean Diana, "the receptionist" means Eve |

"Bring it back" means the last object you mentioned, so naming the object ("bring the contract back") always works.

The office layout, the people and their names, and the objects are all in the `OFFICE` settings in `demo/fetch_demo.py`, so edit them to match your real office. `python demo/fetch_demo.py --world home` switches back to the apartment, with Mom and Dad.

`demo/fetch_demo.py` uses the same robot, and adds a camera with an object detector, a memory of where things are, a path planner and a gripper. You tell it what you want in plain English, and it searches the home, picks the object up and brings it to you (the green "you" dot).

```bash
python demo/fetch_demo.py
```

Type a command in the box at the bottom and press Enter, for example `bring me the cup`, `find my keys`, `where is the remote?`, `explore the house`, `come here` or `stop`. You can also press `1`–`6` to fetch the cup, keys, remote, book, phone or shoes, and `+` / `-` to change the speed.

| Command | What it does |
|---|---|
| `python demo/fetch_demo.py --say "bring me the cup"` | opens the window and starts with that command |
| `python demo/fetch_demo.py --gif fetch.gif --say "bring me the cup" --say "bring me my keys"` | records the run to a GIF |
| `python demo/fetch_demo.py --headless --say "find my phone"` | no window: runs the commands and saves `fetch_result.png` |

How it works, and the ROS 2 equivalent of each piece:

| In the demo | On the real robot |
|---|---|
| `Camera.detect()`: an 80° camera that only sees unoccluded objects within 3.5 m, with noisy scores | a YOLO node on the camera image publishing `vision_msgs/Detection2DArray`, plus depth to get a 3D position |
| `Memory`: an object counts as found after 3 detections, and its position is averaged | a semantic map node that stores labelled poses in the `map` frame |
| `Planner`: a costmap built from the LiDAR map, plus Dijkstra path planning | Nav2 (`BasicNavigator.goToPose`) |
| Search: drive to the spot with the most floor the camera hasn't checked, turn around once, repeat | frontier or viewpoint exploration that sends Nav2 goals |
| `align` → `grasp` → `deliver` | MoveIt 2 for the arm, then a Nav2 goal back to the person |

**Telling it which room.** You can add a room to a command, for example `bring me the cup from the kitchen`, `get my book from the bedroom`, `the keys are on the coffee table`, `go to the hallway` or `explore the kitchen`. Naming a piece of furniture counts as naming its room. The robot drives straight to that room (outlined in orange on its map) and searches only there. If the object isn't there, it says so and searches the rest of the home. If it has already seen the object in a different room, it goes there instead. If it doesn't know the room you name ("the garage"), it says so and searches everywhere. On a real robot, the room labels (`ROOM_SPOTS` and `ROOMS`) are named points and areas you mark once on the saved SLAM map, and "go to the kitchen" is a single Nav2 goal.

**Telling it where to put things.** By default the robot brings things to you, but you can name any room or piece of furniture as the destination: `put the book on the kitchen table`, `take the keys to the bedroom`, `move the cup from the kitchen table to the bed`, `leave my keys on the sofa`. After that, `take it to the kitchen` refers to the last object, and `put it down` drops what it's holding where it is. On furniture, it drives to the closest spot where its arm can reach the top and places the object just inside the edge. In a room, it puts the object on the floor near the room's labelled spot. It refuses places it can't do: a place that isn't on its map ("the garage"), or a surface higher than its arm reaches (the top of the wardrobe). In those cases it brings the object to you instead. The destination is the orange X on both maps.

**Sending things to other people.** Other people in the home can receive things too. The demo has Mom (in the kitchen) and Dad (in the bedroom), set in `PEOPLE` and `PEOPLE_WORDS` in `fetch_demo.py`, so you can rename them or add more. Try `give Mom my keys`, `take the cup to Dad`, `send the book to my mom` or `bring Dad the remote from the sofa`. You start out holding your wallet, so `take this to Mom` makes the robot come to you, take the wallet and deliver it. It always remembers who has what: `where is my wallet?` answers "Mom has the wallet", and `bring me the remote` collects it from whoever has it. If you name someone it doesn't know ("give the book to John"), it says so and brings the object to you.

The detector knows 8 classes (`CLASSES`). Six objects are lying around the house and you are holding the wallet (`OBJECTS`). The glasses are not anywhere, so asking for them makes the robot search the whole home and report that it couldn't find them.

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
4. **Real hardware:** follow [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware); the same SLAM/Nav2 launch files work without `sim:=true`.

## References
- linorobot2: https://github.com/linorobot/linorobot2 · docs: https://linorobot.github.io/linorobot2/
- ROS 2 Jazzy install: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Nav2 setup guides: https://docs.nav2.org/setup_guides/index.html