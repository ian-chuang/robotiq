#include "robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h"

int main(int argc, char** argv)
{
  // Can be renamed with standard ROS-node launch interface
  ros::init(argc, argv, "gripper_2f_gripper_action_server");
  
  // Private Note Handle for retrieving parameter arguments to the server
  ros::NodeHandle private_nh("~");

  std::string action_server_name;
  private_nh.param<std::string>("action_server_name", action_server_name, "gripper_controller/gripper_cmd");

  robotiq_2f_gripper_action_server::Robotiq2FGripperParams cparams;
  private_nh.param<double>("min_angle", cparams.min_angle_, 0.0);
  private_nh.param<double>("max_angle", cparams.max_angle_, 0.8);
  private_nh.param<double>("min_effort", cparams.min_effort_, 20);
  private_nh.param<double>("max_effort", cparams.max_effort_, 235);
  private_nh.param<double>("default_effort", cparams.default_effort_, 100);
  private_nh.param<std::string>("control_topic", cparams.control_topic_, "robotiq_2f_85_gripper/control");
  private_nh.param<std::string>("state_topic", cparams.state_topic_, "robotiq_2f_85_gripper/state");
  private_nh.param<std::string>("joint_states_topic", cparams.joint_states_topic_, "joint_states");
  private_nh.param<std::string>("joint_name", cparams.joint_name_, "finger_joint");

  ROS_INFO("Initializing Robotiq action server for gripper: %s", action_server_name.c_str());

  // The name of the gripper -> this server communicates over name/inputs and name/outputs
  robotiq_2f_gripper_action_server::Robotiq2FGripperActionServer gripper (action_server_name, cparams);

  ROS_INFO("Robotiq action-server spinning for gripper: %s", action_server_name.c_str());
  ros::spin();
}
