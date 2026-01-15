#!/usr/bin/env bash

set -euo pipefail
set +u
source install/setup.bash
set -u
# 예시: 너의 launch로 교체
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  use_fake_hardware:=true \
  robot_ip:=dont-care
