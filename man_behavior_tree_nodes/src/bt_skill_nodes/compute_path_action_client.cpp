#include "man_behavior_tree_nodes/bt_skill_nodes/compute_path_action_client.hpp"

namespace man_behavior_tree_nodes
{

ComputePathActionClient::ComputePathActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait)
: btActionClient<moveit_msgs::MoveGroupAction, 
                  moveit_msgs::MoveGroupGoal>
                  (xml_tag_name, action_name, conf, time_for_wait)
{
}

void ComputePathToPoseAction::on_tick()
{
    // PoseStamped "goal"
    // moveit_msgs::MotionPlanRequest& request;

    getInput("goal", posestamp_);
    getInput("group_name", goal_.request.group_name);
    getInput("replan_times", goal_.request.num_planning_attempts);
    getInput("planner_id", goal_.request.planner_id);
    getInput("workspace_parameters_", goal_.request.workspace_parameters);
    getInput("allowed_planning_time_", goal_.request.allowed_planning_time);
    getInput("max_acceleration_scaling_factor_", goal_.request.max_acceleration_scaling_factor);
    getInput("max_velocity_scaling_factor", goal_.request.max_velocity_scaling_factor);
    getInput("active_target", active_target_);
    getInput("goal_joint_tolerance", goal_joint_tolerance_);
    getInput("joint_model_group", joint_model_group_);


    // Todo: get tolerances
    std::vector<double> position_tolerances(3,0.01f);
    std::vector<double> orientation_tolerances(3,0.01f);

    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(cfg.WRIST_LINK_NAME,posestamp_, position_tolerances,
                                         orientation_tolerances);

    if (!get("arm_state", goal.request.start_state))
        request.start_state.is_diff = true;
        ROS_INFO("No ARM State")

    if (active_target_ == JOINT)
    {
      goal_.request.goal_constraints.resize(1);
      goal_.request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
          getTargetRobotState(), joint_model_group_, goal_joint_tolerance_);
    }
    else if (active_target_ == POSE || active_target_ == POSITION || active_target_ == ORIENTATION)
    {
      // find out how many goals are specified
      std::size_t goal_count = 0;
      for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin();
           it != pose_targets_.end(); ++it)
        goal_count = std::max(goal_count, it->second.size());

      // start filling the goals;
      // each end effector has a number of possible poses (K) as valid goals
      // but there could be multiple end effectors specified, so we want each end effector
      // to reach the goal that corresponds to the goals of the other end effectors
      request.goal_constraints.resize(goal_count);

      for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin();
           it != pose_targets_.end(); ++it)
      {
        for (std::size_t i = 0; i < it->second.size(); ++i)
        {
          moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints(
              it->first, it->second[i], goal_position_tolerance_, goal_orientation_tolerance_);
          if (active_target_ == ORIENTATION)
            c.position_constraints.clear();
          if (active_target_ == POSITION)
            c.orientation_constraints.clear();
          request.goal_constraints[i] = kinematic_constraints::mergeConstraints(request.goal_constraints[i], c);
        }
      }
    }
    else
      ROS_ERROR_NAMED("move_group_interface", "Unable to construct MotionPlanRequest representation");

    if (path_constraints_)
      request.path_constraints = *path_constraints_;
    if (trajectory_constraints_)
      request.trajectory_constraints = *trajectory_constraints_;
  }
    constructMotionPlanRequest(goal_.request)
  

  void constructMotionPlanRequest(moveit_msgs::MotionPlanRequest& request)
  {

    request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
    request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
    request.allowed_planning_time = allowed_planning_time_;

    request.workspace_parameters = workspace_parameters_;

    if (considered_start_state_)
      robot_state::robotStateToRobotStateMsg(*considered_start_state_, request.start_state);
    else
      request.start_state.is_diff = true;

    if (active_target_ == JOINT)
    {
      request.goal_constraints.resize(1);
      request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(
          getTargetRobotState(), joint_model_group_, goal_joint_tolerance_);
    }
    else if (active_target_ == POSE || active_target_ == POSITION || active_target_ == ORIENTATION)
    {
      // find out how many goals are specified
      std::size_t goal_count = 0;
      for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin();
           it != pose_targets_.end(); ++it)
        goal_count = std::max(goal_count, it->second.size());

      // start filling the goals;
      // each end effector has a number of possible poses (K) as valid goals
      // but there could be multiple end effectors specified, so we want each end effector
      // to reach the goal that corresponds to the goals of the other end effectors
      request.goal_constraints.resize(goal_count);

      for (std::map<std::string, std::vector<geometry_msgs::PoseStamped> >::const_iterator it = pose_targets_.begin();
           it != pose_targets_.end(); ++it)
      {
        for (std::size_t i = 0; i < it->second.size(); ++i)
        {
          moveit_msgs::Constraints c = kinematic_constraints::constructGoalConstraints(
              it->first, it->second[i], goal_position_tolerance_, goal_orientation_tolerance_);
          if (active_target_ == ORIENTATION)
            c.position_constraints.clear();
          if (active_target_ == POSITION)
            c.orientation_constraints.clear();
          request.goal_constraints[i] = kinematic_constraints::mergeConstraints(request.goal_constraints[i], c);
        }
      }
    }
    else
      ROS_ERROR_NAMED("move_group_interface", "Unable to construct MotionPlanRequest representation");

    if (path_constraints_)
      request.path_constraints = *path_constraints_;
    if (trajectory_constraints_)
      request.trajectory_constraints = *trajectory_constraints_;
  }


  void constructGoal(moveit_msgs::PickupGoal& goal_out, const std::string& object)
  {
    moveit_msgs::PickupGoal goal;
    goal.target_name = object;
    goal.group_name = opt_.group_name_;
    goal.end_effector = getEndEffector();
    goal.allowed_planning_time = allowed_planning_time_;
    goal.support_surface_name = support_surface_;
    goal.planner_id = planner_id_;
    if (!support_surface_.empty())
      goal.allow_gripper_support_collision = true;

    if (path_constraints_)
      goal.path_constraints = *path_constraints_;

    goal_out = goal;
  }

BT::NodeStatus ComputePathToPoseAction::on_success()
{
  setOutput("path", result_.result->path);

  if (first_time_) {
    first_time_ = false;
  } else {
    config().blackboard->set("path_updated", true);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::ComputePathToPoseAction>(
        name, "compute_path_to_pose", config);
    };

  factory.registerBuilder<nav2_behavior_tree::ComputePathToPoseAction>(
    "ComputePathToPose", builder);
}
