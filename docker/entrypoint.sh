#!/bin/bash
set -e

# 1. 시스템 ROS 소싱
source /opt/ros/humble/setup.bash

# 2. 새로운 터미널(exec)을 위해 .bashrc에도 추가 (중복 방지 체크)
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

exec "$@"
