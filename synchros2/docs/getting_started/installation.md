# Installation

## Binary packages

`synchros2` is released to LTS ROS 2 distributions like Humble (and soon enough Jazzy too), so you can install it from [binaries in Tier 1 platforms](https://docs.ros.org/en/humble/Installation.html#binary-packages). In Ubuntu:

```bash
sudo apt install ros-$ROS_DISTRO-synchros2
```

**Note:** make sure to [setup ROS 2 `apt` sources](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#setup-sources) if you haven't before attempting to install ROS 2 debians!

## Building from source

`synchros2` can also be built from source.

### Prerequisites

1. [Configure a ROS2 environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
2. [Create a ROS2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

**Note:** the links above are to Humble tutorials but should be largely the same for any modern ROS 2 distribution.

### Build `synchros2`

1. Clone `synchros2` into a `colcon` workspace:

    ```bash
    mkdir -p path/to/workspace/src  # for a new workspace
    cd path/to/workspace/src
    git clone https://github.com/bdaiinstitute/synchros2.git
    ```

2. Install `synchros2` dependencies with `rosdep`:

   ```bash
   cd path/to/workspace
   rosdep install -y -i --from-path src
   ```

3. Build and source the `colcon` workspace:

    ```bash
    cd path/to/workspace
    colcon build --symlink-install
    source install/setup.bash
    ```
