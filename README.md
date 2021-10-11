# Komatsu Simulator
3D mining and perception simulator for Komatsu using Unity and ROS 2

# Configuring your system for development

## Unity and ROS2 TCP communications

Follow the instructions from [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-ros2-environment), using the Unity project included in this repo. This will bring you through the ROS2 Environment setup and Unity setup. At the end of the setup, you should be able to run the Unity project and see the it connect to the ROS TCP Endpoint.

## ROS2/Gaia app setup

This section assumes you have the sans libc++ build of Gaia (install .deb from [here](https://drive.google.com/file/d/1GkT4SqxW3cwAwHLpi4nznjWUkFjPWFwp/view?usp=sharing) and ROS2 (Galactic) installed and properly configured.

1. `git clone` this repo someplace.
2. `cd [path_to_where_you_cloned_this_repo]/Ros/src`. You should see 3 directories: `danger_zone`, `danger_zone_msgs`,
   and `retro_log`. Copy these three into your ROS workspace `src` folder.
3. If you don't already have `ros_tcp_endpoint`...
   1. [Git clone the package](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/ROS2) into your ROS workspace `src` folder.
   2. `cd ROS-TCP-Endpoint`
   3. `git fetch --all`
   4. `git checkout ROS2`
4. After sourcing your ROS environment and all your workspaces, run `rosdep install -y -i --from-paths src` at the root
   of your ROS workspace to pickup any dependencies.
5. Run `colcon build` to build all packages.

## Releases
If you prefer to not install and build with Unity, you download the Simulator binaries [here](https://github.com/gaia-platform/KomatsuSimulator/releases)

## Unity setup
1. Install the latest 20x LTS build of the Unity Editor.
2. Follow the Unity instructions [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-unity-setup) to install the Robotics package for Unity.

# Running the simulation

1. On the ROS/Gaia hosting machine...
   1. Source your ROS under/overlays.
   2. Start ROS-TCP-Endpoint:
      `ros2 launch danger_zone bridge_launch.py` 
      - or -
      `ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=[HOST_MACHINE_IP_ADDRESS]`
      1. You can get your host machine's IP address by running `hostname -I`
   3. Start `danger_zone`: `ros2 launch danger_zone node_launch.py`.
   4. On the machine with the Unity simulation, simply start run the simulation and it will automatically connect to ROS. If not, make sure `HOST_MACHINE_IP_ADDRESS` matches the one supplied inside Unity.

