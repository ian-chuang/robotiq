cmake_minimum_required(VERSION 3.0.2)
project(robotiq_2f_gripper_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

catkin_python_setup()

add_message_files(
  FILES
  Robotiq2FGripper_robot_input.msg
  Robotiq2FGripper_robot_output.msg
)

generate_messages(DEPENDENCIES)
catkin_package(
  CATKIN_DEPENDS
  rospy
  message_runtime
)

