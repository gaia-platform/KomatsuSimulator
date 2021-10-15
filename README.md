# Komatsu Simulator
3D mining and perception simulator for Komatsu using Unity and ROS 2

------------------------
# Installation

## Prerequisites

Ubuntu20.04 or later. Other versions of Linux might work but are not supported.

Install in order:

### ROS2

ROS is available in two major versions. ROS and ROS2. ROS2 is available in several releases, KomatsuSimulator requires 'Galatic' or newer.

To install ROS2 Galactic:

1. Follow instructions here: [Installing ROS 2 via Debian Packages](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

2. Add the ROS environment to your shell init script `.bashrc` (not required but highly recommended). If you have a previous version of ROS installed be careful to not source them both (eg. remove `source /opt/ros/foxy/setup.bash` from your `~/bashrc`).
    ```bash
    echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc
    ```

    Or, if you use a different shell such as .zsh:

    ```zsh
    echo  'source /opt/ros/galactic/setup.zsh' >> ~/.zshrc
    ```

3. Install colcon for compiling ROS2 projects:
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

4. Install python3-rosdep2 for managing ROS2 dependencies:
    ```bash
    sudo apt install python3-rosdep2
    ```

    After installation, execute:
    ```bash
    rosdep update
    ```

Knowledge of ROS2 is not required in order to use KomatsuSim at the most basic level. More information on ROS2 can be found at [ROS](https://www.ros.org/).

### ROS-TCP-Endpoint

To install ROS-TCP-Endpoint:

1. Create a ROS workspace folder structure (https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html). We suggest `~/ros2_ws/src/`. Further instructions will assume that location.
   ```bash
   mkdir -p ~/ros2_ws/src
   ```

2. Move into the `~/ros2_ws/src` directory:

    ```bash
    cd ~/ros2_ws/src
    ```

3. clone ROS2 branch of https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git into the `~/ros2_ws/src` directory:

    ```bash
    git clone -b ROS2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
    ```

4. Move to the `~/ros2_ws` directory:

    ```bash
    cd ~/ros2_ws
    ```

5. Build the `~/ros2_ws` directory:

    ```bash
    colcon build
    ```

6. Source the `~/ros2_ws` directory in .bashrc (not required but highly recommended):

    ```bash
    echo 'source ~/ros2_ws/install/local_setup.bash' >> ~/.bashrc
    ```

### KomatsuSimulator

KomatsuSimulator has two projects: 1) a Unity based simulator and 2) a ROS project. In order to use KomatsuSimulator one must always install ROS2 and Git the KomatsuSimulator repo. If one prefers to avoid building the Unity simulator then one may download the GaiaMineSimVx.x.x.x.zip binary from one of the releases, typically the latest release. Downloading and using the simulator binary does not eliminate the need to install ROS2 and Git the KomatsuSimulator repo. The releases can be found here: https://github.com/gaia-platform/KomatsuSimulator/releases

To install KomatsuSimulator:

1. Create a directory for development. Here we will assume it is '~/dev', feel free to use any location you wish.
    ```bash
    mkdir ~/dev
    ```

2. Move into the the `~/dev` directory.
    ```bash
    cd ~/dev
    ```

3. Git the KomatsuSimulator repo.
    ```bash
    git clone https://github.com/gaia-platform/KomatsuSimulator.git
    ```

4. Move into the KomatsuSimulator ROS2 workspace.
    ```bash
    cd ~/dev/KomatsuSimulator/Ros
    ```

5. Install ROS project's depedencies.
    ```bash
    rosdep install -y -i --from-paths .
    ```

6. Build the `~/dev/KomatsuSimulator/Ros` directory.
    ```bash
    colcon build
    ```

7. Source the `~/dev/KomatsuSimulator/Ros` directory in .bashrc (not required but highly recommended).
    ```bash
    echo 'source ~/dev/KomatsuSimulator/Ros/install/local_setup.bash' >> ~/.bashrc
    ```

8. To avoid building the Unity simulator:
   1. Download GaiaMineSimVx.x.x.x.zip from the latest release at https://github.com/gaia-platform/KomatsuSimulator/releases
   2. Unzip in a location of your choice
   3. Run GaiaMineSim to verify operation.
       ```bash
       ./Builds/GaiaMine.x86_64
       ```

# USE

There are several levels of development possible. It is possible to work in only one level, with little to no involvement in the others.

1. Develop only Gaia schema and rules.
2. Develop danger zone ROS2 code.
3. Develop GaiaMineSim Simulator environments, game objects, and code.

## 1. Develop only Gaia schema and rules

1. Start ros-unity bridge.
   - Open a terminal.
   - Start ROS-TCP-Endpoint:
    ```bash
    ros2 launch danger_zone bridge_launch.py
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

3. Start danger zone.
   - Open a terminal.
   - Start danger_zone:
    ```bash
    ros2 launch danger_zone node_launch.py
    ```
   - Leave terminal open.

4. Now everything is running. Watch the simulator run, look at ROS messages, etc.

5. Stop danger_zone.
   - Go in the danger_zone terminal: `<ctrl>C`.

The Gaia schema and rules are in the danger_zone ROS2 project. Standard ROS2 dev practices apply.

## Releases
If you prefer to not install and build with Unity, you can download the Simulator binaries [here](https://github.com/gaia-platform/KomatsuSimulator/releases)

## Unity setup
1. Install the latest 20x LTS build of the Unity Editor.
2. Follow the Unity instructions [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-unity-setup) to install the Robotics package for Unity.
