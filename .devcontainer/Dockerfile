FROM osrf/ros:iron-desktop

# non interactive frontend for locales
ENV DEBIAN_FRONTEND=noninteractive

# installing texlive and utils
RUN apt-get update && \
    apt-get -y install \
    ament-cmake \
    build-essential \
    clang-13 \
    cloc \
    doxygen \
    gdb \
    git \
    iwyu \
    lcov \
    locales \
    python3-pip \
    python3-colcon-lcov-result \
    && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install rosdep opencv-python

# Set Locales
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Source ROS2
RUN echo "source /opt/ros/iron/setup.bash" >> /root/.bashrc
