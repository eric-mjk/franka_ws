#!/usr/bin/env bash
source install/setup.bash

# 예시: 너의 launch로 교체
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  use_fake_hardware:=true \
  robot_ip:=dont-care