#!/usr/bin/env bash
# One command per terminal. Run each in its own terminal window/tab.
#
#   bash scripts/run.sh gazebo [world]   1) start Gazebo + linorobot2 (world: turtlebot3_world | playground)
#   bash scripts/run.sh check            2) verify /scan, /odom and the TF tree are alive
#   bash scripts/run.sh slam             3) start SLAM Toolbox + RViz (map builds live)
#   bash scripts/run.sh explore          4) let home_explorer drive the robot around
#   bash scripts/run.sh teleop             ...or drive it yourself with the keyboard
#   bash scripts/run.sh savemap [name]   5) save the map to ros2_ws/maps/<name>.yaml
#   bash scripts/run.sh nav [map.yaml]   6) Nav2 on a saved map (click "2D Goal Pose" in RViz)
#
# Add --soft as the LAST argument if Gazebo shows a black/blank window
# (common under WSL2 without GPU drivers): forces software rendering.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"

if [[ "${*: -1}" == "--soft" ]]; then
  export LIBGL_ALWAYS_SOFTWARE=1
  set -- "${@:1:$(($#-1))}"
fi

# shellcheck disable=SC1091
cmd="${1:-help}"
case "$cmd" in
  gazebo|check|slam|explore|teleop|savemap|nav) ;;
  *) sed -n '2,13p' "$0"; exit 0 ;;
esac
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "!! ROS 2 Jazzy not found. Run: bash scripts/1_install_ros2_jazzy.sh"; exit 1
fi
source /opt/ros/jazzy/setup.bash
if [[ ! -f "$WS/install/setup.bash" ]]; then
  echo "!! Workspace not built yet. Run: bash scripts/2_setup_workspace.sh"; exit 1
fi
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
export LINOROBOT2_BASE="${LINOROBOT2_BASE:-2wd}"

shift || true
case "$cmd" in
  gazebo)
    WORLD="${1:-turtlebot3_world}"
    # spawn_x 0.5: the turtlebot3 world has a pillar at the origin
    exec ros2 launch linorobot2_gazebo gazebo.launch.py world_name:="$WORLD" spawn_x:=0.5
    ;;
  check)
    echo "== Topics =="
    ros2 topic list | grep -E "^/(scan|odom|odometry/filtered|cmd_vel|imu/data|tf|clock)$" || true
    echo; echo "== /scan rate (5 s) =="
    timeout 5 ros2 topic hz /scan --window 20 || true
    echo; echo "== One /scan message summary =="
    timeout 5 ros2 topic echo --once /scan --field header || echo "!! no /scan received"
    echo; echo "== TF odom -> base_footprint =="
    timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>/dev/null | head -n 6 \
      || echo "!! no odom -> base_footprint transform yet"
    echo; echo "Tip: 'ros2 run tf2_tools view_frames' writes frames.pdf with the whole TF tree."
    ;;
  slam)
    exec ros2 launch linorobot2_navigation slam.launch.py sim:=true rviz:=true
    ;;
  explore)
    echo ">> Pause anytime:  ros2 param set /explorer enabled false"
    exec ros2 launch home_explorer explore.launch.py sim:=true
    ;;
  teleop)
    echo ">> If the explorer is running, pause it first: ros2 param set /explorer enabled false"
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ;;
  savemap)
    NAME="${1:-home_map}"
    mkdir -p "$WS/maps"
    ros2 run nav2_map_server map_saver_cli -f "$WS/maps/$NAME" \
      --ros-args -p save_map_timeout:=10000.0 -p use_sim_time:=true
    echo "✅ Saved $WS/maps/$NAME.yaml (+ .pgm)"
    ;;
  nav)
    MAP="${1:-}"
    if [[ -z "$MAP" ]]; then
      exec ros2 launch linorobot2_navigation navigation.launch.py sim:=true rviz:=true
    fi
    exec ros2 launch linorobot2_navigation navigation.launch.py sim:=true rviz:=true map:="$(realpath "$MAP")"
    ;;
  *)
    sed -n '2,13p' "$0"
    ;;
esac
