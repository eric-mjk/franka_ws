# docker/franka.Dockerfile
FROM franka-ros2:local

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive

# (이미지 공통) 개발용 기본 툴만 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-vcstool \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# rosdep은 이미지에 한 번만 세팅해두면 편함
RUN rosdep init || true && rosdep update

# 기본 작업 디렉토리만 지정 (볼륨이 덮어써도 OK)
WORKDIR /workspace

CMD ["bash"]
