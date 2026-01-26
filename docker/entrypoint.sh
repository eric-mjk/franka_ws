#!/usr/bin/env bash
set -e

# ROS 2 base
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# workspace overlay (if built)
if [ -f /workspaces/franka_ws/install/setup.bash ]; then
  source /workspaces/franka_ws/install/setup.bash
fi

# sudo apt-get update && rosdep install --from-paths src --ignore-src -r -y

exec "$@"
EOF

chmod +x docker/entrypoint.sh