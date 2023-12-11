# Mobile Grasping Robot
 This repository is specifically created to compile documentation and code related to Team 8's project. The project focuses on building an autonomous mobile robot using Leo Rover Kits, which includes components like the Trossen Robotics Pincher X150 mobile arm, Slamtec RpLiDAR A2M12, and Intel D435 RealSense depth camera. The robot is designed to carry out various tasks, including grasping, navigation, and other functionalities. Please note that the project is still in progress, so the code is incomplete.

## Table of Contents

1. [Installation](#installation)
2. [Future Scope](#future-scope)
3. [Leo Navigation](#leo-navigation)
## Installation
Please refer to the documentation provided on the [Leo Rover offical website](https://www.leorover.tech/knowledge-base) for the assembly process and other information regards leo rover.

This installation procedure assumes that you have installed ROS2 Humble and [Gazebo (preferably ignition version)](https://gazebosim.org/api/gazebo/6.1/install.html) based on Ubuntu 22.04:

1. The first step is the installation of the leo_simulator. Access and clone the [leo simulator-ros2](https://github.com/LeoRover/leo_simulator-ros2). You can also go to official ROS website of [leo gazebo](http://wiki.ros.org/leo_gazebo).

2. Create a python package and sourcing its setup file. 
```
ros2 pkg create --build-type ament_python <package_name>
source install/local_setup.bash
```
3. In the third step, clone our repository and position it within your package. Once again, follow the standard procedure of compiling the Python package and sourcing its setup file, as previously mentioned.

## Future Scope

* **`navigation algorithm`** To achieve full autonomy for Rover, we can employ techniques such as semantic segmentation and reinforcement learning to further enhance the learning capabilities of our Leo Rover.
* **`grasping`** Grasping involves both object recognition and grasping. We will employ various methods rooted in deep learning.The initial strategy aims to achieve robotic arm movement using Gazebo MoveIt and process images using OpenCV.
 [Gazebo MoveIt](https://github.com/bjsowa/interbotix_ros_arms/tree/master) and process images using OpenCV. 
## Leo Navigation
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
