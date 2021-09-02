# Komatsu Simulator
3D mining and perception simulator for Komatsu using Unity and ROS 2

# Configuring your system for development

## Unity and ROS2 TCP communications

Follow the instructions from [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-ros2-environment), using the Unity project included in this repo. This will bring you through the ROS2 Environment setup and Unity setup. At the end of the setup, you should be able to run the Unity project and see the it connect to the ROS TCP Endpoint.

## ROS2/Gaia app setup

This section assumes you have Gaia (March release) and ROS2 (Foxy) installed and properly configured.

1. `git clone` this repo someplace.
2. `cd [path_to_where_you_cloned_this_repo]/Ros/src`. You should see 3 directories: `danger_zone`, `danger_zone_msgs`,
   and `retro_log`. Copy these three into your ROS workspace `src` folder.
3. If you don't already have `visions_msgs`...
    1. [Git clone the package](https://github.com/ros-perception/vision_msgs.git) into your ROS workspace `src` folder.
    2. `cd vision_msgs`
    3. `git fetch --all`
    4. `git checkout ros2`
4. After sourcing your ROS environment and all your workspaces, run `rosdep install -y -i --from-paths src` at the root
   of your ROS workspace to pickup any dependencies.
5. Run `colcon build --packages-select [name_of_package]` several times, replacing `[name_of_package]` with these
   packages in this order:
    1. `vision_msgs`
    2. `danger_zone_msgs`
    3. `danger_zone`
    4. `retro_log`

* Special Gaia.Preview build instructions 

As of 20210901 if building against Gaia.Preview, as special sans libc++ build of GaiaPlatform is needed. 
This is not an official or supported realease, it is just here for a limited time to work around the Gaia/ROS build issue.
The build can be found at: https://drive.google.com/file/d/1GkT4SqxW3cwAwHLpi4nznjWUkFjPWFwp/view?usp=sharing 
