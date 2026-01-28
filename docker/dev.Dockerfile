# docker/dev.Dockerfile
FROM franka-ros2-franka:local

WORKDIR /workspace

# ROS 자동 source
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# (선택) colcon auto-complete 같은거 하고 싶으면 여기에 추가
# RUN apt-get update && apt-get install -y --no-install-recommends <something> && rm -rf /var/lib/apt/lists/*

CMD ["bash"]
