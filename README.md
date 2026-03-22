# This repository is terminated

Check following repository.

https://github.com/eric-mjk/Moveit_Dual_FR3 


# franka_ws

ROS 2 workspace for Franka FR3 robot control and motion planning with MoveIt 2.

## Contents

- **control** — custom robot control package
- **franka_ros2** — Franka ROS 2 driver
- **moveit2** — MoveIt 2 motion planning
- **moveit_task_constructor** — task-level motion planning (MTC)
- **franka_description** — robot URDF/SRDF descriptions

## Branches

- `main` — primary development
- `t-franka` — early single-arm workspace
- `t2-franka` — second iteration workspace

## Docker

A Docker-based dev environment is provided under `docker/`.

```bash
docker compose -f docker/docker-compose.yml up
```

## Requirements

- ROS 2 (Humble or later)
- libfranka
- MoveIt 2

