# docker/libfranka.Dockerfile
# ✅ 너의 기존 이미지 태그로 바꾸기
FROM eric.moveit2:latest

ENV DEBIAN_FRONTEND=noninteractive

# (중요) 혹시 예전에 ros-humble-libfranka(0.15)가 깔려있으면 제거해서 충돌 방지
RUN apt-get update && \
    apt-get remove -y ros-humble-libfranka || true

# (중요) /usr/local에 예전 소스빌드 잔재가 있으면 제거 (있을 때만 영향)
RUN rm -rf /usr/local/include/franka || true && \
    rm -f /usr/local/lib/libfranka* || true && \
    ldconfig

# libfranka 0.19.0 .deb 설치 (Jammy)
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget ca-certificates && \
    rm -rf /var/lib/apt/lists/*

RUN wget -q https://github.com/frankarobotics/libfranka/releases/download/0.19.0/libfranka_0.19.0_jammy_amd64.deb && \
    wget -q https://github.com/frankarobotics/libfranka/releases/download/0.19.0/libfranka_0.19.0_jammy_amd64.deb.sha256 && \
    sha256sum -c libfranka_0.19.0_jammy_amd64.deb.sha256 && \
    dpkg -i libfranka_0.19.0_jammy_amd64.deb || (apt-get update && apt-get install -f -y) && \
    rm -f libfranka_0.19.0_jammy_amd64.deb*
