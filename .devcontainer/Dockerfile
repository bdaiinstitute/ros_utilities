FROM ros:humble-ros-base-jammy

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ARG ROS_DISTRO=humble
ENV ROS_DISTRO $ROS_DISTRO
ARG INSTALL_PACKAGE=base

# Install packages
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN DEBIAN_FRONTEND=noninteractive apt-get update -q; \
    apt-get update -q; \
    apt-get install -yq --no-install-recommends \
        python3-pip \
        python-is-python3 \
        python3-argcomplete \
        python3-colcon-common-extensions \
        python3-colcon-mixin \
        python3-pytest-cov \
        python3-rosdep \
        libpython3-dev \
        python3-vcstool; \
    rm -rf /var/lib/apt/lists/*

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"

# ROS Packages
RUN apt-get update -q; \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-action-tutorials-interfaces \
        ros-${ROS_DISTRO}-example-interfaces; \
    rm -rf /var/lib/apt/lists/*

# Install python packages inside the new environment
RUN python -m pip install --no-cache-dir --upgrade pip==22.3.1; \
    pip install --root-user-action=ignore --no-cache-dir --default-timeout=900 \
      numpy==1.24.1; \
    pip cache purge

# ROS doesn't recognize the docker shells as terminals so force colored output
ENV RCUTILS_COLORIZED_OUTPUT=1
