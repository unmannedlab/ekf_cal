# @todo Replace with minimal ROS build and incorporate rosdep
FROM osrf/ros:humble-desktop

# non interactive frontend for locales
ENV DEBIAN_FRONTEND=noninteractive

# installing texlive and utils
RUN apt-get update && \
    apt-get -y install \
    git \
    locales && \
    rm -rf /var/lib/apt/lists/*

# Set Locales
RUN sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Source ROS2
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc