# docker/ubuntu.Dockerfile
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
    curl \
    git \
    build-essential \
    cmake \
    sudo \
    locales \
    && rm -rf /var/lib/apt/lists/*

# locale (ROS 필수)
RUN locale-gen en_US en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y \
    curl ca-certificates \
    git build-essential cmake sudo locales python3 \
    && rm -rf /var/lib/apt/lists/*
