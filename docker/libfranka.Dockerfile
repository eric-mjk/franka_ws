# docker/libfranka.Dockerfile
FROM eric.moveit2:latest

ENV DEBIAN_FRONTEND=noninteractive

# (중요) 혹시 예전에 ros-humble-libfranka(0.15)가 깔려있으면 제거해서 충돌 방지
RUN apt-get update && \
    apt-get remove -y ros-humble-libfranka || true

# (중요) /usr/local에 예전 소스빌드 잔재가 있으면 제거 (있을 때만 영향)
RUN rm -rf /usr/local/include/franka || true && \
    rm -f /usr/local/lib/libfranka* || true && \
    ldconfig

# =========================
# libfranka 0.15.0 (source build) + robotpkg pinocchio
# https://github.com/frankarobotics/libfranka/tree/0.15.0
# =========================

# build deps + robotpkg repo setup + pinocchio install
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential cmake git \
      libpoco-dev libeigen3-dev libfmt-dev \
      lsb-release curl ca-certificates \
    && mkdir -p /etc/apt/keyrings \
    && curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
      | tee /etc/apt/keyrings/robotpkg.asc >/dev/null \
    && echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
      | tee /etc/apt/sources.list.d/robotpkg.list >/dev/null \
    && apt-get update \
    && apt-get install -y --no-install-recommends robotpkg-pinocchio \
    && rm -rf /var/lib/apt/lists/*

# build & install libfranka 0.15.0
RUN git clone --recurse-submodules https://github.com/frankaemika/libfranka.git /tmp/libfranka && \
    cd /tmp/libfranka && \
    git checkout 0.15.0 && \
    git submodule update --init --recursive && \
    mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake \
          -DBUILD_TESTS=OFF \
          .. && \
    make -j"$(nproc)" && \
    make install && \
    ldconfig && \
    rm -rf /tmp/libfranka