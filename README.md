# Komatsu Simulator
3D mining and perception simulator for Komatsu using Unity and ROS 2

# Configuring your system for development

## Unity and ROS2 TCP communications

Follow the instructions from [here](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md#-ros2-environment), using the Unity project included in this repo. This will bring you through the ROS2 Environment setup and Unity setup. At the end of the setup, you should be able to run the Unity project and see the it connect to the ROS TCP Endpoint.

## ROS2/Gaia app setup

This section assumes you have Gaia (March release) and ROS2 (Foxy) installed and properly configured.

1. [Git clone the app](https://github.com/gaia-platform/gaia_detect3d_ros.git) into your ROS workspace `src` folder.
2. Compile the code as a Gaia app. 
3. After sourcing your ROS environment or your workspace, run `rosdep install -y -i --from-paths src` to pickup any dependencies.
4. Run `colcon build` to build the app.
5. If you get an error saying `vision_msgs` is missing...
   1. [Git clone the package](https://github.com/ros-perception/vision_msgs.git) into your ROS workspace `src` folder.
   2. `cd [path_to_ROS_ws]/src/vision_msgs`
   3. `git fetch --all`
   4. `git checkout ros2`
   5. Then `colcon build` again.
   6. If the error persists, source your workspace again and build once more.



