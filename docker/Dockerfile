FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
SHELL ["/bin/bash", "-lc"]

# -----------------------------
# 0) (옵션) Ubuntu apt 미러
# -----------------------------
RUN sed -i 's/archive.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirror.kakao.com/g' /etc/apt/sources.list

# -----------------------------
# 1) Base utilities + locale
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales tzdata \
    ca-certificates curl wget gnupg2 lsb-release \
    git build-essential cmake pkg-config \
    python3-pip python3-venv python3-dev \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# -----------------------------
# 2) ROS2 Humble apt repo
# -----------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

# -----------------------------
# 3) Install ROS2 + dev tools
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# 기본 ROS source
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# -----------------------------
# 4) ROS deps commonly needed by franka_ros2
#    (너가 겪었던 control_msgs 포함)
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-control-msgs \
    ros-humble-realtime-tools \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 5) libfranka 설치 전략 선택
#    - 기본: 소스 빌드 (재현성/호환성 ↑)
#    - 필요 시: 시스템 패키지로 대체 가능하게 플래그
# -----------------------------
ARG LIBFRANKA_FROM_SOURCE=1
ARG LIBFRANKA_VERSION=0.13.2

RUN if [ "${LIBFRANKA_FROM_SOURCE}" = "1" ]; then \
      set -eux; \
      apt-get update && apt-get install -y --no-install-recommends \
        libpoco-dev libeigen3-dev \
      && rm -rf /var/lib/apt/lists/*; \
      mkdir -p /tmp/libfranka && cd /tmp/libfranka; \
      git clone --recurse-submodules --shallow-submodules \
        --depth 1 --branch v${LIBFRANKA_VERSION} \
        https://github.com/frankaemika/libfranka.git \
      || git clone --recurse-submodules --shallow-submodules \
        --depth 1 --branch ${LIBFRANKA_VERSION} \
        https://github.com/frankaemika/libfranka.git; \
      cd libfranka; \
      git submodule update --init --recursive; \
      cmake -S . -B build -DCMAKE_BUILD_TYPE=Release; \
      cmake --build build -j$(nproc); \
      cmake --install build; \
      ldconfig; \
      rm -rf /tmp/libfranka; \
    else \
      echo "LIBFRANKA_FROM_SOURCE=0 selected. (You must ensure libfranka is installed via apt in another way)"; \
    fi


# -----------------------------
# 6) RViz/MoveIt GUI (X11/GL)
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    mesa-utils \
    libgl1-mesa-dri \
    libgl1 \
    libx11-6 libxext6 libxrender1 libxtst6 \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# 7) rosdep init/update (1회)
# -----------------------------
RUN rosdep init || true && rosdep update

# -----------------------------
# 8) Workspace 준비 (COPY는 최대한 뒤로)
# -----------------------------
WORKDIR /workspaces/franka_ws

# 여기서부터 소스 변경 시 캐시가 깨짐 (의도적으로 맨 뒤에 둠)
COPY src ./src

# rosdep으로 의존성 설치 (distro 고정)
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
      --skip-keys="olv_module_descriptions" && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------
# 9) 개발 사용자
# -----------------------------
ARG USERNAME=mj
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get update && apt-get install -y --no-install-recommends sudo \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME} \
    && rm -rf /var/lib/apt/lists/*

# entrypoint
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
