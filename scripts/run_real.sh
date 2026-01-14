#!/usr/bin/env bash
set -e

cd "$(dirname "$0")/.."

set -a
source .env
set +a

source install/setup.bash

: "${ROBOT_IP:?ROBOT_IP not set in .env}"

ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true