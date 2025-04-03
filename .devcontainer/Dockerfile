ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN DEBIAN_FRONTEND=noninteractive apt-get update -q && \
    apt-get dist-upgrade -y --no-install-recommends && \
    apt-get install -yq --no-install-recommends \
    python3-pip \
    python-is-python3 \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-pytest-cov \
    python3-rosdep \
    python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN source "/opt/ros/${ROS_DISTRO}/setup.bash"

RUN --mount=type=bind,source=.,target=/tmp/context \
    apt-get update -q && rosdep update && \
    rosdep install -y -i --from-paths /tmp/context && \
    rm -rf /var/lib/apt/lists/*

ENV RCUTILS_COLORIZED_OUTPUT=1
