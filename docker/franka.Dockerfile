# docker/franka.Dockerfile
FROM franka-ros2:local

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-lc"]

# Tools required for vcs + builds
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-vcstool \
 && rm -rf /var/lib/apt/lists/*

# Workspace location inside the image
ENV FRANKA_WS=/opt/franka_ros2_ws

RUN source /opt/ros/humble/setup.bash && \
    mkdir -p ${FRANKA_WS}/src && \
    cd ${FRANKA_WS}/src && \
    if [ ! -d "franka_ros2" ]; then \
        echo ">>> franka_ros2 not found, cloning..."; \
        git clone https://github.com/frankarobotics/franka_ros2.git franka_ros2; \
    else \
        echo ">>> franka_ros2 already exists, skip clone"; \
    fi
    
RUN vcs import ${FRANKA_WS}/src < ${FRANKA_WS}/src/franka_ros2/dependency.repos --recursive --skip-existing
RUN rosdep update
RUN apt-get update
RUN rosdep install --from-paths ${FRANKA_WS}/src --ignore-src --rosdistro humble -y
