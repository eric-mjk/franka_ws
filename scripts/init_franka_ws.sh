#!/usr/bin/env bash
set -euo pipefail
cd /workspaces/franka_ws

echo "[franka] Importing franka_ws.repos..."
mkdir -p src
vcs import src < franka_ws.repos

echo "[franka] Importing franka_ros2 dependency.repos..."
vcs import src/franka_ros2 < src/franka_ros2/dependency.repos --recursive --skip-existing

echo "[franka] apt-get update (required before rosdep install)..."
apt-get update

echo "[franka] Installing rosdep dependencies..."
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

echo "[franka] Workspace is ready. Run colcon build manually."
