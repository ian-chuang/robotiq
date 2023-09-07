/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq 2F gripper
 */

#include "robotiq_2f_gripper_action_server/robotiq_2f_gripper_action_server.h"
#include <sensor_msgs/JointState.h>

// To keep the fully qualified names managable

//Anonymous namespaces are file local -> sort of like global static objects
namespace
{
  using namespace robotiq_2f_gripper_action_server;

  /*  This struct is declared for the sole purpose of being used as an exception internally
      to keep the code clean (i.e. no output params). It is caught by the action_server and 
      should not propogate outwards. If you use these functions yourself, beware.
  */
  struct BadArgumentsError {};

  double clip(double n, double lower, double upper) {
    return std::max(lower, std::min(n, upper));
  }

  GripperOutput goalToRegisterState(const GripperCommandGoal& goal, const Robotiq2FGripperParams& params)
  {
    GripperOutput result;
    result.rACT = 0x1; // active gripper
    result.rGTO = 0x1; // go to position
    result.rATR = 0x0; // No emergency release
    result.rSP = 128; // Middle ground speed

    double position = goal.command.position;
    double max_effort = goal.command.max_effort;

    if (max_effort == 0) {
      max_effort = params.default_effort_;
    }
    
    if (position > params.max_angle_ || position < params.min_angle_)
    {
      ROS_WARN("Goal gripper angle is out of range(%f to %f): %f m",
               params.min_angle_, params.max_angle_, position);
      throw BadArgumentsError();
    }
    
    if (max_effort < params.min_effort_ || max_effort > params.max_effort_)
    {
      ROS_WARN("Goal gripper effort out of range (%f to %f N): %f N",
               params.min_effort_, params.max_effort_, max_effort);
      throw BadArgumentsError();
    }

    double dist_per_tick = (params.max_angle_ - params.min_angle_) / 230;
    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    result.rPR = static_cast<uint8_t>((position - params.min_angle_) / dist_per_tick);
    result.rFR = static_cast<uint8_t>((max_effort - params.min_effort_) / eff_per_tick);

    ROS_INFO("Setting goal position register to %hhu", result.rPR);

    return result;
  }

  /*  This function is templatized because both GripperCommandResult and GripperCommandFeedback consist
      of the same fields yet have different types. Templates here act as a "duck typing" mechanism to avoid
      code duplication.
  */
  template<typename T>
  T registerStateToResultT(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    T result;
    double dist_per_tick = (params.max_angle_ - params.min_angle_) / 230;
    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    result.position = clip(input.gPO * dist_per_tick + params.min_angle_, params.min_angle_, params.max_angle_);    
    result.effort = input.gCU * eff_per_tick + params.min_effort_;
    result.stalled = input.gOBJ == 0x1 || input.gOBJ == 0x2;
    result.reached_goal = input.gPO == goal_pos;

    return result;
  }

  // Inline api-transformers to avoid confusion when reading the action_server source
  inline
  GripperCommandResult registerStateToResult(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandResult>(input, params, goal_pos);
  }

  inline
  GripperCommandFeedback registerStateToFeedback(const GripperInput& input, const Robotiq2FGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandFeedback>(input, params, goal_pos);
  }

} // end of anon namespace

namespace robotiq_2f_gripper_action_server
{

Robotiq2FGripperActionServer::Robotiq2FGripperActionServer(const std::string& name, const Robotiq2FGripperParams& params)
  : nh_()
  , as_(nh_, name, false)
  , action_name_(name)
  , gripper_params_(params)
{
  as_.registerGoalCallback(boost::bind(&Robotiq2FGripperActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&Robotiq2FGripperActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe(gripper_params_.state_topic_, 1, &Robotiq2FGripperActionServer::analysisCB, this);
  goal_pub_ = nh_.advertise<GripperOutput>(gripper_params_.control_topic_, 1);
  joint_pub = nh_.advertise<sensor_msgs::JointState>(gripper_params_.joint_states_topic_, 10);
  
  std::vector<double> joint_positions(1);
  std::vector<std::string> joint_names(1, "");
  joint_names.at(0).assign(gripper_params_.joint_name_);
  joint_msg.name = joint_names;
  joint_msg.position = joint_positions;

  ROS_INFO("Waiting for gripper control node");
  ros::WallDuration sleep_t(0.02); 
  while (goal_pub_.getNumSubscribers() < 1 && ros::ok()) {
    sleep_t.sleep();
  }  

  as_.start();

  ROS_INFO("Gripper action server ready");
}

void Robotiq2FGripperActionServer::goalCB()
{
  // Check to see if the gripper is in an active state where it can take goals
  if (current_reg_state_.gSTA != 0x3)
  {
    ROS_WARN("%s could not accept goal because the gripper is not yet active", action_name_.c_str());
    return;
  }

  GripperCommandGoal current_goal (*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested())
  {
    as_.setPreempted();
  }

  try
  {
    goal_reg_state_ = goalToRegisterState(current_goal, gripper_params_);
    goal_pub_.publish(goal_reg_state_);
  }
  catch (BadArgumentsError& e)
  {
    ROS_INFO("%s No goal issued to gripper", action_name_.c_str());
  }
}

void Robotiq2FGripperActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void Robotiq2FGripperActionServer::analysisCB(const GripperInput::ConstPtr& msg)
{
  current_reg_state_ = *msg;

  publishJointStates(msg);

  if (!as_.isActive()) return;

  // Check to see if the gripper is in its activated state
  if (current_reg_state_.gSTA != 0x3)
  {
    // Check to see if the gripper is active or if it has been asked to be active
    if (current_reg_state_.gSTA == 0x0 && goal_reg_state_.rACT != 0x1)
    {
      issueActivation();
    }
    return;
  }

  // check for errors
  if (current_reg_state_.gFLT) 
  {
    ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.gFLT);
    as_.setAborted(registerStateToResult(current_reg_state_,
                                         gripper_params_,
                                         goal_reg_state_.rPR));
  } 
  // If commanded to move and if at a goal state and if the position request matches the echo'd PR, we're done with a move
  else if (current_reg_state_.gGTO && current_reg_state_.gOBJ && current_reg_state_.gPR == goal_reg_state_.rPR) 
  {
    ROS_INFO("%s succeeded", action_name_.c_str());
    as_.setSucceeded(registerStateToResult(current_reg_state_,
                                           gripper_params_,
                                           goal_reg_state_.rPR));
  }
  // Publish feedback
  else 
  {
    as_.publishFeedback(registerStateToFeedback(current_reg_state_,
                                                gripper_params_,
                                                goal_reg_state_.rPR));
  }
}

void Robotiq2FGripperActionServer::publishJointStates(const GripperInput::ConstPtr& msg) {
  current_reg_state_ = *msg;

  double dist_per_tick = (gripper_params_.max_angle_ - gripper_params_.min_angle_) / 230;
  double position = clip(current_reg_state_.gPO * dist_per_tick + gripper_params_.min_angle_, gripper_params_.min_angle_, gripper_params_.max_angle_);

  joint_msg.header.frame_id = "";
  joint_msg.header.stamp = ros::Time::now();
  joint_msg.position.at(0) = position;
  joint_pub.publish(joint_msg);
}

void Robotiq2FGripperActionServer::issueActivation()
{
  ROS_INFO("Activating gripper for gripper action server: %s", action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x1;
  out.rGTO = 0x0; // go to position
  out.rATR = 0x0; // No emergency release
  out.rSP = 0X0; // Middle ground speed
  out.rPR = 0x0; // position
  out.rFR = 0X0; // effort
  goal_reg_state_ = out;
  goal_pub_.publish(out);
}

void Robotiq2FGripperActionServer::issueReset()
{
  ROS_INFO("Resetting gripper for gripper action server: %s", action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x0;
  out.rGTO = 0x0; // go to position
  out.rATR = 0x0; // No emergency release
  out.rSP = 0x0; // Middle ground speed
  out.rPR = 0x0; // position
  out.rFR = 0x0; // effort
  goal_reg_state_ = out;
  goal_pub_.publish(out);
}


} // end robotiq_2f_gripper_action_server namespace
