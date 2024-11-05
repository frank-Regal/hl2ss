FROM ubuntu:22.04

ENV ROS_DISTRO=humble

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y && \
    apt install -y software-properties-common

RUN apt update && add-apt-repository universe

RUN apt update && apt install -y curl && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update && apt upgrade && apt install -y ros-${ROS_DISTRO}-ros-base ros-dev-tools

RUN apt update && apt upgrade -y && apt install -y \
    apt-utils \
    curl \
    git \
    python3-pip \
    python3-argcomplete \
    python3-colcon-common-extensions \
    build-essential \
    libgflags-dev \
    libzmq3-dev \
    libeigen3-dev \
    libgl1-mesa-glx \
    vim \
    wget \
    make \
    sudo \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
ENV TERM=xterm-256color
RUN echo -e "PS1='\e[1m\e[93m\u\e[97m@\e[93m\h:\e[35m\w\e[0m\e[97m#\e[0m '" >> ~/.bashrc

# setup .bashrc to correctly source workspace and setup user aliases
SHELL ["/bin/bash", "-l", "-c"]


# copy the entrypoint into the image
COPY ./entrypoint.sh /entrypoint.sh

# run this script on startup
ENTRYPOINT /entrypoint.sh
