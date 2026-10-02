#!/usr/bin/env bash
# Step 2 - put linorobot2 next to home_explorer in ros2_ws/src and build.
#
# Usage:   bash scripts/2_setup_workspace.sh [2wd|4wd|mecanum]
set -eo pipefail

BASE="${1:-2wd}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$ROOT/ros2_ws"

if [[ "$ROOT" == /mnt/* ]]; then
  echo "!! The project is on the Windows drive ($ROOT)."
  echo "   Builds there are very slow under WSL2. Copy it into Linux first:"
  echo "     cp -r \"$ROOT\" ~/ && cd ~/$(basename "$ROOT") && bash scripts/2_setup_workspace.sh"
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
cd "$WS"

echo ">> Cloning linorobot2 (jazzy) and linorobot2_viz into $WS/src"
[[ -d src/linorobot2 ]] || git clone -b jazzy --depth 1 https://github.com/linorobot/linorobot2 src/linorobot2
[[ -d src/linorobot2_viz ]] || git clone --depth 1 https://github.com/linorobot/linorobot2_viz src/linorobot2_viz

echo ">> Installing dependencies with rosdep"
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src -y \
  --skip-keys microxrcedds_agent --skip-keys micro_ros_agent

echo ">> Building (first build takes a few minutes)"
colcon build --symlink-install

# environment for every new terminal
if ! grep -q "LINOROBOT2_BASE" ~/.bashrc; then
  echo "export LINOROBOT2_BASE=$BASE" >> ~/.bashrc
fi
if ! grep -q "$WS/install/setup.bash" ~/.bashrc; then
  echo "source $WS/install/setup.bash" >> ~/.bashrc
fi

echo
echo "✅ Workspace built. Open a NEW terminal (or run: source ~/.bashrc), then:"
echo "   bash scripts/run.sh gazebo     # terminal 1"
echo "   bash scripts/run.sh slam       # terminal 2"
echo "   bash scripts/run.sh explore    # terminal 3"
