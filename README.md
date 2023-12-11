# Mobile Grasping Robot
 This repository is specifically created to compile documentation and code related to Team 8's project. The project focuses on building an autonomous mobile robot using Leo Rover Kits, which includes components like the Pincher X150 mobile arm, RpLiDAR A2M12, and Intel D435 RealSense depth camera. The robot is designed to carry out various tasks, including grasping, navigation, and other functionalities.

## Table of Contents

1. [Installation](#installation)
2. [Future Scope](#future-scope)
3. [Launch Files](#launch-files)
4. [License](#license)

## Installation
Please refer to the documentation provided on the [official Leo Rover website](https://www.leorover.tech/knowledge-base) for the assembly process.

This installation procedure assumes that you have installed ROS2 Humble and [Gazebo (preferably version ignition)](https://gazebosim.org/api/gazebo/6.1/install.html) based on Ubuntu 22.04:

1. The first step is the installation of the leo_simulator. You have to go to this link and clone the [leo simulator](https://github.com/LeoRover/leo_simulator) github repo. You can also go to official ROS website of leo gazebo. You can also go to official ROS website of [leo gazebo](http://wiki.ros.org/leo_gazebo).

2. Create a python package and sourcing its setup file. 
```
ros2 pkg create --build-type ament_python <package_name>
source install/local_setup.bash
```
3. In the third step you will clone our repo and place it in your package and again repeat the customary step of compiling the python package and sourcing its setup file as stated above. Note the project is not complete yet. 

## Future Scope

* **`navigation algorithm`** Autonomous driving is widely recognized for leveraging the capabilities of machine learning. To achieve full autonomy for Rover, we can employ techniques such as semantic segmentation and reinforcement learning to further enhance the learning capabilities of our Leo Rover.
* **`grasping realization`** Including object recognition and grasping. We will choose some methods based on deep learning. The preliminary approach is to realize the movement of the robotic arm by gazebo moveit and image processing by OpenCv.

## Launch files
Based on leo_navigation tutorials. The refine is not complete yet.
* **`odometry.launch`** 
 
    Starts the `ekf_localization_node` from [robot_localization] which publishes the odometry based on Wheel encoders and IMU readings.

    **Arguments:**
    * `three_d` (default: `false`)
    
        If set to `true`, also starts [imu_filter_madgwick] to fuse data from IMU sensor into an orientation and uses it in the `ekf_localization_node` to provide a 3D odometry.

* **`gmapping.launch`**

    Starts the `slam_gmapping` node from [gmapping] package which provides a laser-based SLAM.

* **`amcl.launch`** 

    Starts [map_server] which publishes static map from a file on a ROS topic and [amcl] which uses odometry and data from the LiDAR sensor to estimate the localization of the robot on the map.

    **Arguments:**
    * `map_file` (**required**)

        An absolute path to the map file in the format supported by `map_server`.

* **`twist_mux.launch`**

    Starts [twist_mux] node which multiplexes several sources of velocity commands for the robot, giving priority to manual control over autonomous.

    **Arguments:**
    * `cmd_vel_out` (default: `cmd_vel`)

        The topic name the multiplexer should publish velocity commands on.

* **`move_base.launch`**

    Starts [move_base] node which, given the robot's localization, a map of obstacles, laser scans and a navigation goal, plans a safe path to the goal and tries to execute it, by sending velocity commands to the robot.

* **`navigation.launch`**

    Starts `twist_mux.launch` and `move_base.launch`.

[Autonomous Navigation tutorial]: https://www.leorover.tech/guides/autonomous-navigation
[geometry_msgs/TwistWithCovarianceStamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html
[sensor_msgs/Imu]: http://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
[geometry_msgs/TwistStamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistStamped.html
[geometry_msgs/Vector3Stamped]: http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html
[leo_firmware]: https://github.com/LeoRover/leo_firmware
[robot_localization]: http://wiki.ros.org/robot_localization
[imu_filter_madgwick]: http://wiki.ros.org/imu_filter_madgwick
[gmapping]: http://wiki.ros.org/gmapping
[amcl]: http://wiki.ros.org/amcl
[map_server]: http://wiki.ros.org/map_server
[twist_mux]: http://wiki.ros.org/twist_mux
[move_base]: http://wiki.ros.org/move_base

## License
MIT License

Copyright (c) 2020-2021 Kell Ideas Ltd.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
