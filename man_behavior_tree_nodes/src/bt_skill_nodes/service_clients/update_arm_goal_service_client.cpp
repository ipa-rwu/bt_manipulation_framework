#include "man_behavior_tree_nodes/bt_skill_nodes/service_clients/update_arm_goal_service_client.hpp"

namespace man_behavior_tree_nodes
{

UpdateArmGoalServiceClient::UpdateArmGoalServiceClient(
  const std::string & xml_tag_name,
  const std::string & service_name,
  const BT::NodeConfiguration & conf): 
  btServiceClient<man_msgs::UpdateArmGoal>(xml_tag_name, service_name, conf)
{
}

void UpdateArmGoalServiceClient::on_tick()
{
  update_param_ = 0;
  // If param provided, will update goal
  if(!getInput("param", update_param_))
  {
    update_param_ = 0;
  }
  else
  {
    config().blackboard->get<double>("recovery_arm_parameter", old_param_);
    update_param_ += old_param_;
  }

  // If task step provoided, will get param from blackboard
  if(getInput("step", step_))
  {
    config().blackboard->get<std::map<std::string, float>>("param_float", param_float_);
    config().blackboard->set<std::string>("current_step", step_); 
    update_param_ += param_float_.find(step_)->second;
  }

  config().blackboard->set<double>("recovery_arm_parameter", update_param_);
  service_.request.param = update_param_;
    
  if(!getInput("initial_pose", service_.request.target)){
    ROS_ERROR_STREAM_NAMED(service_client_name_, service_client_name_ <<": Please provide initial goal!!");
  }

  // frame id of the goal of the arm
  if(!getInput("goal_frame_id", service_.request.frame_id)){
    config().blackboard->get<std::string>("world_frame_id", world_frame_id_); 
    service_.request.frame_id = world_frame_id_;
  }
} 

BT::NodeStatus UpdateArmGoalServiceClient::on_success()
{
  setOutput("goal", service_.response.goal);
  return BT::NodeStatus::SUCCESS;
}
}   //namespace

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::UpdateArmGoalServiceClient>(
        name, "update_arm_goal", config);
    };

  factory.registerBuilder<man_behavior_tree_nodes::UpdateArmGoalServiceClient>(
    "UpdateGoalForArm", builder);
}