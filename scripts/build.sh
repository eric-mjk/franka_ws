#!/usr/bin/env bash

cd "$(dirname "$0")/.."

colcon build --symlink-install
