#!/usr/bin/env bash

cd "$(dirname "$0")/.."

# Import external repos
if [ -f franka.repos ]; then
  vcs import src < franka.repos
fi

# Install deps
rosdep update
rosdep install --from-paths src --ignore-src -r -y
