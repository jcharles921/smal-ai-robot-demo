# Home Robot Starter

A small, working starting point for home robotics with **ROS 2 Jazzy + Gazebo + linorobot2**, in two parts:

| Part | What it is | Needs | Time to first run |
|---|---|---|---|
| **A. `demo/`** | A 2D robot that explores an apartment with a simulated LiDAR and builds a map live | Python 3 only | ~1 minute |
| **B. `ros2_ws/` + `scripts/`** | The real thing: linorobot2 in Gazebo, SLAM Toolbox, Nav2, plus your own ROS 2 node (`home_explorer`) that drives the robot autonomously | Ubuntu 24.04 (native or WSL2) | ~30–60 min (mostly downloads) |

Part A uses **the same exploration algorithm** as the ROS 2 node in part B, so what you see in the demo is what the robot does in Gazebo.

```
home_robot_starter/
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

## A. Run the demo right now

**Windows:** install Python from python.org (tick *Add python.exe to PATH*), then double-click `demo\run_demo.bat`.

**Linux / macOS / WSL:**
```bash
bash demo/run_demo.sh
```
(or `pip install numpy matplotlib` then `python3 demo/sim_demo.py`)

You get two panels: the left is the "real" world (like Gazebo) with LiDAR beams in red; the right is the map the robot builds from those beams (like the `/map` that SLAM Toolbox publishes). The status bar shows the `/cmd_vel` it is sending.

| Key | Action |
|---|---|
| `m` | toggle AUTO / MANUAL |
| arrow keys | drive in MANUAL (↑↓ speed, ←→ turn) |
| space | stop |
| `r` | reset robot and map |
| `p` | save the map to `map.png` |
| `q` | quit |

No window available (SSH, server)? `python3 demo/sim_demo.py --headless` saves `demo_result.png` after 4 simulated minutes. In testing it maps about 80% of the apartment in 5 simulated minutes without hitting anything.

**Things to try:** edit `build_world()` to draw your own home, or change `ExplorerController`'s `stop_dist` / `max_speed` and watch how behaviour changes.

---

## B. The full ROS 2 + Gazebo + linorobot2 setup

linorobot2's current branch is **`jazzy`**, which means **Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**.

### 0. Windows only: get Ubuntu 24.04 via WSL2
In PowerShell (as administrator, once):
```powershell
wsl --install -d Ubuntu-24.04
```
Reboot if asked, open "Ubuntu 24.04" from the Start menu and create a user. Windows 11 shows Linux GUI apps (Gazebo, RViz) automatically.

Copy this project **into the Linux side** (building on `/mnt/c` is very slow):
```bash
cp -r /mnt/c/Users/<YOU>/Downloads/home_robot_starter ~/
cd ~/home_robot_starter
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
