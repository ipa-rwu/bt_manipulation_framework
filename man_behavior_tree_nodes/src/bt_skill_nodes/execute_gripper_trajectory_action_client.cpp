#include "man_behavior_tree_nodes/bt_skill_nodes/execute_gripper_trajectory_action_client.hpp"

namespace man_behavior_tree_nodes
{

ExecuteGripperTrajectoryActionClient::ExecuteGripperTrajectoryActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait)
: btActionClient<man_msgs::ExecuteGripperTrajectoryAction, 
                man_msgs::ExecuteGripperTrajectoryGoal,
                man_msgs::ExecuteGripperTrajectoryResultConstPtr>
                  (xml_tag_name, action_name, conf, time_for_wait)
{
}

void ExecuteGripperTrajectoryActionClient::on_tick()
{
    // PoseStamped "goal"
    // moveit_msgs::MotionPlanRequest& request;

    if(!getInput("action_name", goal_.action_name))
    {
        if(!getInput("step", step_))
             throw BT::RuntimeError("ExecuteGripperTrajectoryActionClient missing required input [step]");
        else
        {
         config().blackboard->get<std::string>("current_step", step_); 
        }
        
        
        // ROS_INFO_STREAM_NAMED("ExecuteGripperTrajectoryActionClient", "[ExecuteGripper] step: "<< step_);
     
        config().blackboard->get<std::map<std::string, std::string>>("param_string", param_string_);
        goal_.action_name = param_string_.find(step_)->second;
        // ROS_INFO_STREAM_NAMED("ExecuteGripperTrajectoryActionClient", "[ExecuteGripper] target: "<< goal_.action_name);
    }


}
  
BT::NodeStatus ExecuteGripperTrajectoryActionClient::on_success()
{
    success_ = result_->success;
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
      return std::make_unique<man_behavior_tree_nodes::ExecuteGripperTrajectoryActionClient>(
        name, "execute_gripper_trajectory", config, time_for_wait);
    };

  factory.registerBuilder<man_behavior_tree_nodes::ExecuteGripperTrajectoryActionClient>(
    "ExecuteTrajectoryGripper", builder);
}