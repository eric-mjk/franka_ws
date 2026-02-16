# docker/ros2.Dockerfile
FROM eric.ubuntu:latest
ENV DEBIAN_FRONTEND=noninteractive

# Make apt more reliable in Docker builds
RUN set -eux; \
    printf 'Acquire::Retries "10";\nAcquire::http::Timeout "60";\nAcquire::https::Timeout "60";\n' \
      > /etc/apt/apt.conf.d/80-retries; \
    sed -i 's|http://archive.ubuntu.com/ubuntu|https://archive.ubuntu.com/ubuntu|g' /etc/apt/sources.list; \
    sed -i 's|http://security.ubuntu.com/ubuntu|https://security.ubuntu.com/ubuntu|g' /etc/apt/sources.list


# Add ROS2 apt repo + key
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl ca-certificates gnupg \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}}) main" \
    > /etc/apt/sources.list.d/ros2.list \
 && rm -rf /var/lib/apt/lists/*

# Upgrade before ROS install (Jammy warning)
RUN apt-get update && apt-get upgrade -y \
 && apt-get install -y --only-upgrade udev systemd \
 && rm -rf /var/lib/apt/lists/*

# Install ROS2
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    python3-rosdep python3-colcon-common-extensions ros-dev-tools \
 && rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y --no-install-recommends --fix-missing \
    ros-humble-desktop \
 && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update
