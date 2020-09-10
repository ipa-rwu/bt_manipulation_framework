#include "man_behavior_tree_nodes/bt_skill_nodes/execute_trajectory_arm_action_client.hpp"

namespace man_behavior_tree_nodes
{

ExecuteTrajectoryActionClient::ExecuteTrajectoryActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait)
: btActionClient<man_msgs::ExecuteTrajectorySkillAction, 
                man_msgs::ExecuteTrajectorySkillGoal,
                man_msgs::ExecuteTrajectorySkillResultConstPtr>
                  (xml_tag_name, action_name, conf, time_for_wait)
{
}

void ExecuteTrajectoryActionClient::on_tick()
{
    // PoseStamped "goal"
    // moveit_msgs::MotionPlanRequest& request;

    getInput("plan", goal_.plan);

}
  
BT::NodeStatus ExecuteTrajectoryActionClient::on_success()
{
    success_ = result_->success;
    // for debug
    setOutput("result", success_);

    if(success_ == 1)
        return BT::NodeStatus::SUCCESS;

    if(success_ == 0)
        return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  float time_for_wait = 20.0;
  BT::NodeBuilder builder =
    [&time_for_wait](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::ExecuteTrajectoryActionClient>(
        name, "execute_trajectory_arm", config, time_for_wait);
    };

  factory.registerBuilder<man_behavior_tree_nodes::ExecuteTrajectoryActionClient>(
    "ExecuteTrajectoryArm", builder);
}