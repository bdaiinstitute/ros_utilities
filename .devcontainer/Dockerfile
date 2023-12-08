FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# install packages
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

ARG ROS_DISTRO=humble
ENV ROS_DISTRO $ROS_DISTRO
ARG INSTALL_PACKAGE=base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN DEBIAN_FRONTEND=noninteractive apt-get update -q && \
    apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pytest-cov \
    python3-rosdep \
    libpython3-dev \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# I added this
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Ros Example Packages
RUN apt-get update -q && \
    apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-action-tutorials-interfaces \
    ros-${ROS_DISTRO}-example-interfaces && \
    rm -rf /var/lib/apt/lists/*

# Install packages inside the new environment
RUN python -m pip install --no-cache-dir --upgrade pip==22.3.1 \
    && pip install --root-user-action=ignore --no-cache-dir --default-timeout=900 \
    numpy==1.24.1 \
    && pip cache purge

# Install dependencies for repository packages
RUN --mount=type=bind,source=.,target=/tmp/context \
    apt-get update -q && rosdep update && \
    rosdep install -y -i --from-paths /tmp/context && \
    rm -rf /var/lib/apt/lists/*

# ROS doesn't recognize the docker shells as terminals so force colored output
ENV RCUTILS_COLORIZED_OUTPUT=1
