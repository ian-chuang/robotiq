## Robotiq ROS Driver for 2F-85 Gripper

This repository provides the ROS driver for controlling the Robotiq 2F-85 Gripper. The package includes the URDF model of the 2F-85 Gripper with Gazebo support. Additionally, it features a Gripper Action Server that can be easily utilized by MoveIt to control the gripper.

This repository is a fork of the original [jr-robotics/robotiq](https://github.com/jr-robotics/robotiq) repository. Only the necessary packages for the 2F-85 Gripper have been retained and modified, while support for the 2F-140 and 3F grippers has been removed.

![Robotiq Real](media/robotiq_real.gif)

### Features

- Added I/O coupling to the gripper URDF.
- The gripper URDF includes an optional cable protector that is compatible with I/O coupling.
- Gazebo support for the gripper URDF with correct 4-bar linkage on both fingers and mimic joint plugin.
- Fixed minor bugs and errors in the action server and Modbus RTU from the original fork to ensure proper functionality with the real gripper.

![Robotiq Gazebo](media/robotiq_gazebo.gif)

### Compatibility

This driver has only been tested on ROS Noetic.

### Installation

To get started, follow these steps:

1. Install the main Python dependency, pymodbus version 2.1.0, using the following command:

    ```
    pip3 install pymodbus===2.1.0
    ```

2. Clone this repository:

    ```
    git clone https://github.com/ian-chuang/robotiq
    ```

3. Optionally, if you wish to control both fingers in Gazebo using the mimic joint plugin, clone the mimic joint plugin repository:

    ```
    git clone https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins.git
    ```

4. Navigate to your ROS workspace's source directory and install dependencies:

    ```
    cd ~/<your_workspace>/src
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO}
    ```

5. Build the workspace:

    ```
    cd ~/<your_workspace>
    catkin build
    ```

### Usage

1. Control the Real Gripper using either Publisher Subscriber or Gripper Action Server:

    ```
    roslaunch robotiq_2f_gripper_control robotiq_2f_85_gripper_rtu.launch port:=<port defaults to /tmp/ttyUR>
    ```

2. Visualize the Gripper URDF in RViz:

    ```
    roslaunch robotiq_2f_85_gripper_description view_robotiq_2f_85.launch
    ```

Feel free to explore and integrate this ROS driver to efficiently control and simulate the Robotiq 2F-85 Gripper in your projects. For additional details, refer to the original [jr-robotics/robotiq](https://github.com/jr-robotics/robotiq) repository.