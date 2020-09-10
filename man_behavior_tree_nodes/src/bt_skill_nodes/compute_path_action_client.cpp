#include "man_behavior_tree_nodes/bt_skill_nodes/compute_path_action_client.hpp"

namespace man_behavior_tree_nodes
{

ComputePathActionClient::ComputePathActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait)
: btActionClient<man_msgs::ComputePathSkillAction,
                  man_msgs::ComputePathSkillGoal,
                  man_msgs::ComputePathSkillResultConstPtr>
                  (xml_tag_name, action_name, conf, time_for_wait)
{
}

void ComputePathActionClient::on_tick()
{
    // PoseStamped "goal"
    // moveit_msgs::MotionPlanRequest& request;

    getInput("goal", goal_.goal);

    config().blackboard->get<std::string>("group_name_arm", goal_.group_name);
    config().blackboard->get<std::string>("end_effector", goal_.end_effector);


    
    if(!getInput("target_type", goal_.target_type))
       throw BT::RuntimeError("ComputePath Action missing required input [target_type]");

    if(!getInput("goal_name", goal_.named_goal))
      goal_.named_goal = named_goal_;
        
    if(!getInput("replan_times", goal_.num_planning_attempts))
      goal_.num_planning_attempts = replan_times_;

    if(!getInput("planner_id", goal_.planner_id))
      goal_.planner_id = planner_id_;
    
    if(!getInput("allowed_planning_time", goal_.allowed_planning_time))
      goal_.allowed_planning_time = allowed_planning_time_;
    
    if(!getInput("max_acceleration_scaling_factor", goal_.max_acceleration_scaling_factor))
      goal_.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;

    if(!getInput("max_velocity_scaling_factor", goal_.max_velocity_scaling_factor))
      goal_.max_velocity_scaling_factor = max_velocity_scaling_factor_;

    if(!getInput("eef_step", goal_.eef_step))
      goal_.eef_step = eef_step_;

    if(!getInput("jump_threshold", goal_.jump_threshold))
      goal_.jump_threshold = jump_threshold_;
    
    if(!getInput("position_tolerances", goal_.position_tolerances))
      goal_.position_tolerances = position_tolerances_;
    
    if(!getInput("orientation_tolerances", goal_.orientation_tolerances))
      goal_.orientation_tolerances = orientation_tolerances_;

    if(!getInput("is_attached", goal_.is_attached))
      goal_.is_attached = is_attached_;
}
  
BT::NodeStatus ComputePathActionClient::on_success()
{
  setOutput("plan", result_->plan);
  // setOutput("trajectory_start", result_.result->trajectory_start);
  // setOutput("trajectory", result_.result->trajectory);
  // setOutput("planning_time", result_.result->planning_time);
  // setOutput("group_name", result_.result->group_name);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  float time_for_wait = 20.0;
  BT::NodeBuilder builder =
    [&time_for_wait](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::ComputePathActionClient>(
        name, "compute_path", config, time_for_wait);
    };

  factory.registerBuilder<man_behavior_tree_nodes::ComputePathActionClient>(
    "ComputePathArm", builder);
}