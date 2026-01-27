# docker/dev.Dockerfile
FROM franka-ros2

WORKDIR /workspace

# ROS 자동 source
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 여기에 프로젝트 의존성만 추가
# RUN apt-get install -y ...

CMD ["bash"]
