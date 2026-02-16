# docker/dev.Dockerfile
FROM eric.ros2:latest

# 2. 환경 변수 설정 (터미널 설치 중 인터랙티브 질문 방지)
ENV DEBIAN_FRONTEND=noninteractive

# 3. 필수 도구 및 자동 완성 설치
RUN apt-get update && apt-get install -y \
    bash-completion \
    sudo \
    curl \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

# 4. Bash 자동 완성 활성화 (사용자 .bashrc에 추가)
RUN echo "source /etc/profile.d/bash_completion.sh" >> /etc/bashrc \
    && echo "source /usr/share/bash-completion/bash_completion" >> ~/.bashrc

# 5. 작업 디렉토리 설정 (docker-compose의 working_dir와 일치)
WORKDIR /workspaces/franka_ws

# 6. 기본 실행 명령 (컨테이너가 바로 종료되지 않게 bash 유지)
CMD ["bash"]