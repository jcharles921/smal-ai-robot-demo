#!/usr/bin/env bash
# Step 1 - install ROS 2 Jazzy + Gazebo Harmonic on Ubuntu 24.04
# (native Ubuntu, or Ubuntu 24.04 inside WSL2 on Windows 11).
#
# Usage:   bash scripts/1_install_ros2_jazzy.sh
# Takes 10-30 min depending on your internet. Needs ~5 GB of disk.
set -euo pipefail

source /etc/os-release
if [[ "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "!! This script expects Ubuntu 24.04 (noble); you have ${PRETTY_NAME:-unknown}."
  echo "   ROS 2 Jazzy + linorobot2 'jazzy' branch target 24.04."
  echo "   On Windows:  wsl --install -d Ubuntu-24.04"
  exit 1
fi

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  echo ">> ROS 2 Jazzy already installed - only adding Gazebo/teleop extras."
else
  echo ">> Locale"
  sudo apt-get update
  sudo apt-get install -y locales curl software-properties-common
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8

  echo ">> ROS 2 apt repository"
  sudo add-apt-repository -y universe
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                           | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb

  echo ">> ROS 2 Jazzy desktop + dev tools (big download)"
  sudo apt-get update
  sudo apt-get upgrade -y
  sudo apt-get install -y ros-jazzy-desktop ros-dev-tools
fi

echo ">> Gazebo Harmonic bridge, SLAM, Nav2, teleop"
sudo apt-get install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-slam-toolbox \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-teleop-twist-keyboard \
  python3-numpy python3-matplotlib git

echo ">> rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

if ! grep -q "/opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo
echo "✅ ROS 2 Jazzy installed. Next:  bash scripts/2_setup_workspace.sh"
