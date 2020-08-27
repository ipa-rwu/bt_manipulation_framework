#include "man_behavior_tree_nodes/bt_skill_nodes/update_arm_goal_service_client.hpp"

namespace man_behavior_tree_nodes
{

UpdateArmGoalServiceClient::UpdateArmGoalServiceClient(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
: btServiceClient<man_msgs::UpdateArmGoal>(xml_tag_name, service_name, conf)
{
}

void UpdateArmGoalServiceClient::on_tick()
{
    config().blackboard->get<std::map<std::string, float>>("param_float", param_float_);
    getInput("step", step_);
    getInput("goal_frame_id", service_.request.frame_id);
    getInput("target", service_.request.target);
    
    // ROS_INFO( "request: \"%s\", \"%s\"", request_->topic.c_str(), request_->data_type.c_str());
    

    service_.request.param = param_float_.find(step_)->second;

    ROS_INFO_STREAM_NAMED("update_goal_for_arm", "[req] step: "<< step_ << "  param: "<< service_.request.param);
    ROS_INFO_STREAM_NAMED("update_goal_for_arm", "[req] target: "<< service_.request.target);
    ROS_INFO_STREAM_NAMED("update_goal_for_arm", "[req] goal_frame_id: "<< service_.request.frame_id);
    
}  // namespace man_behavior_tree_nodes

BT::NodeStatus UpdateArmGoalServiceClient::on_success()
{
    ROS_INFO("get response");
    setOutput("goal", service_.response.goal);
    ROS_INFO_STREAM_NAMED("update_goal_for_arm", "[response] goal: "<< service_.response.goal);
    return BT::NodeStatus::SUCCESS;
}
} //namespace


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::UpdateArmGoalServiceClient>(
        name, "update_goal", config);
    };

  factory.registerBuilder<man_behavior_tree_nodes::UpdateArmGoalServiceClient>(
    "UpdateGoalForArm", builder);
}