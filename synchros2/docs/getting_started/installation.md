# Installation

## Prerequisites

These links are to Humble tutorials but `synchros2` should be compatible with any current ROS2 distro.

1. [Configure a ROS2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
2. [Create a ROS2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

## Install and Build `synchros2`

1. Download the git repository into your existing ROS workspace:
    
    ```bash
    cd <workspace>/src
    git clone https://github.com/bdaiinstitute/ros_utilities.git
    ```
    
2. Build and source the colcon workspace
    
    ```bash
    cd <workspace>
    colcon build --symlink-install
    source install/setup.bash
    ```
