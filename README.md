# Komatsu Simulator
3D mining and perception simulator for Komatsu using Unity and ROS 2

------------------------
# Installation

## Prerequisites

Ubuntu 20.04 or later. Other versions of Linux might work but are not supported.

Install in order:

### ROS2

ROS is available in two major versions: ROS and ROS2. ROS2 is available in several releases. KomatsuSimulator requires 'Galatic' or newer.

To install ROS2 Galactic:

1. Follow instructions here: [Installing ROS 2 via Debian Packages](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

2. Add the ROS environment to your shell init script, `.bashrc` (not required but highly recommended). If a previous version of ROS is installed be careful to not source them both (eg. remove any `source /opt/ros/<version>/setup.bash` lines from your `~/bashrc`).
    ```bash
    echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc
    ```

    Or, if using a different shell such as .zsh:

    ```zsh
    echo  'source /opt/ros/galactic/setup.zsh' >> ~/.zshrc
    ```

3. Install all necessary dependencies
    ```bash
    sudo apt install python3-colcon-common-extensions python3-rosdep2 python3-vcstool
    ```

    After installation, initialize and update `rosdep` with:
    ```bash
    sudo rosdep init
    rosdep update
    ```

Knowledge of ROS2 is not required in order to use KomatsuSim at the most basic level. More information on ROS2 can be found at [ROS](https://www.ros.org/).

### Create and Build a ROS2 Workspace

1. Create a ROS2 workspace folder structure (https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html). We suggest `~/ros2_ws/src/`. Further instructions will assume that location.
   ```bash
   mkdir -p ~/ros2_ws/src
   ```

2. Move into the `~/ros2_ws/src` directory:

    ```bash
    cd ~/ros2_ws/src
    ```

3. Clone the required repositories to your ROS2 workspace:

    ```bash
    git clone https://github.com/gaia-platform/danger_zone.git
    git clone https://github.com/gaia-platform/rosbag2_snapshot.git
    git clone -b ROS2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
    ```

4. Install all necessary dependencies using `rosdep`:

   ```bash
   rosdep install --from-paths . -i -y
   ```

5. Build the `~/ros2_ws` directory:

    ```bash
    cd ..
    colcon build
    ```

6. Source the `~/ros2_ws` directory:

    Option 1: add to your ~/.bashrc once (highly recommended):

    ```bash
    echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
    source ~/.bashrc
    ```

    Option 2: source the directory in each new terminal:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

### KomatsuSimulator

KomatsuSimulator has two projects: 1) a Unity-based simulator and 2) a ROS2 project.
The instructions above handle the ROS2 components but it is also necessary to download the KomatsuSimulator Unity project.
If it is preferred to avoid building the Unity simulator then the GaiaMineSimVx.x.x.x.zip binary can be downloaded from [the releases page](https://github.com/gaia-platform/KomatsuSimulator/releases).
GAIA typically recommends downloading the latest release.
Downloading and using the simulator binary does not eliminate the need to follow the ROS2 instructions above, it just avoids having to build the Unity project from source.

#### Downloading and Using the Binary

1. Download GaiaMineSimVx.x.x.x.zip from the latest release at https://github.com/gaia-platform/KomatsuSimulator/releases
2. Unzip in a location of your choice
3. Run GaiaMineSim to verify operation.
   ```bash
   ./Builds/GaiaMine.x86_64
   ```

#### Cloning and Building from Source

1. Install the latest 20x LTS build of the Unity Editor.
2. Follow the Unity instructions [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-unity-setup) to install the Robotics package for Unity.

3. Create a directory for development. Here we will assume it is '~/dev', but feel free to use any location you wish.
    ```bash
    mkdir ~/dev
    ```

4. Move into the the `~/dev` directory.
    ```bash
    cd ~/dev
    ```

5. Clone the KomatsuSimulator repo.
    ```bash
    git clone https://github.com/gaia-platform/KomatsuSimulator.git
    ```

6. Enter the KomatsuSimulator folder
    ```bash
    cd KomatsuSimulator
    ```

6. TODO(markwest): Please fill in build instructions when using source

# Usage

## Startup all nodes at once

You can use the `full_launch.py` script to launch all the ROS nodes at once. 

1. Start all the ROS nodes
   - Open a terminal.
   - Call the full_launch.py script
        ```bash
        ros2 launch danger_zone full_launch.py 
        ```
   - Leave terminal open.

2. Start simulator.
   - Open terminal
   - Move to install location of simulator
   - Start the simulator:
        ```bash
        ./Builds/GaiaMine.x86_64
        ```
   - Leave terminal open.

## Startup nodes independently

1. Start ros-unity bridge.
   - Open a terminal.
   - Start ROS-TCP-Endpoint:
        ```bash
        ros2 launch danger_zone bridge_launch.py
        ```
   - Leave terminal open.

2. Start the snapshotter.
   - Open a terminal.
   - Start snapshotter:
        ```bash
        ros2 run rosbag2_snapshot snapshotter
        ```
   - Leave terminal open.

3. Start simulator.
   - Open terminal
   - Move to install location of simulator
   - Start the simulator:
        ```bash
        ./Builds/GaiaMine.x86_64
        ```
   - Leave terminal open.

4. Start danger zone.
   - Open a terminal.
   - Start danger_zone:
        ```bash
        ros2 launch danger_zone node_launch.py
        ```
   - Leave terminal open.

5. Now everything is running. Watch the simulator run, look at ROS messages, etc.

6. Stop danger_zone.
   - Go in the danger_zone terminal:
        ```
        <ctrl>C
        ```
# Development

There are several levels of development possible. It is possible to work in only one level, with little to no involvement in the others.

1. Develop only Gaia schema and rules.
2. Develop danger zone ROS2 code.
3. Develop GaiaMineSim Simulator environments, game objects, and code.

The Gaia schema and rules are in the [`danger_zone` ROS2 project](https://github.com/gaia-platform/danger_zone) which should be in the ROS2 workspace created earlier (e.g. `~/ros2_ws/src/danger_zone`). Standard [ROS2 developer best-practices](https://docs.ros.org/en/rolling/Contributing/Developer-Guide.html) apply.

If you wish to develop the KomatsuSimulator Unity application, see the instructions above on Cloning and Building from Source.
