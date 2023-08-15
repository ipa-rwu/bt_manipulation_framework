// Copyright (c) 2023 Ruichao Wu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "moveit_skills/action/compute_path_action_server_core.hpp"

namespace moveit_skills
{
template <class ActionT>
void ComputePathActionServerCore<ActionT>::initPlanComponentParameters(
  moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params,
  const RosParameters & parameters)
{
  plan_params = moveit_cpp::PlanningComponent::PlanRequestParameters();
  if (parameters.planner_id != "") {
    plan_params.planner_id = parameters.planner_id;
  }
  if (parameters.planning_pipeline != "") {
    plan_params.planning_pipeline = parameters.planning_pipeline;
  }
  plan_params.planning_attempts = parameters.planning_attempts;
  plan_params.planning_time = parameters.planning_time;
  plan_params.max_velocity_scaling_factor = parameters.max_velocity_scaling_factor;
  plan_params.max_acceleration_scaling_factor = parameters.max_acceleration_scaling_factor;
}

template <class ActionT>
void ComputePathActionServerCore<ActionT>::setPlanner(
  moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params,
  const std::string & planning_pipeline, const std::string & planner_id)
{
  RCLCPP_INFO(*logger_, "initPlanComponentParameters");

  if (!planning_pipeline.empty()) {
    plan_params.planning_pipeline = planning_pipeline;
  }
  if (!planner_id.empty()) {
    plan_params.planner_id = planner_id;
  }
  RCLCPP_INFO(
    *logger_, "setPlanner: Using planning_pipeline: %s, planner_id: %s",
    plan_params.planning_pipeline.c_str(), plan_params.planner_id.c_str());
}

template <class ActionT>
bool ComputePathActionServerCore<ActionT>::plan(
  const moveit_cpp::MoveItCppPtr & moveit_cpp_ptr,
  planning_scene_monitor::PlanningSceneMonitorPtr psm,
  moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params,
  const moveit::core::RobotState & target_robot_state, const moveit::core::JointModelGroup * jmg,
  const moveit_msgs::msg::Constraints & path_constraints, double goal_joint_tolerance,
  robot_trajectory::RobotTrajectoryPtr & result)
{
  bool success = false;

  // Create planning component
  auto planning_components =
    std::make_shared<moveit_cpp::PlanningComponent>(jmg->getName(), moveit_cpp_ptr);

  auto start_robot_state = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(
    planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState(), start_robot_state);

  auto target_robot_state_msg = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(target_robot_state, target_robot_state_msg);

  // set Path Constraints
  planning_components->setPathConstraints(path_constraints);

  std::vector<moveit_msgs::msg::Constraints> goal_constraints;

  goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints(target_robot_state, jmg, goal_joint_tolerance));

  // Copy goal constraint into planning component
  planning_components->setStartStateToCurrentState();
  planning_components->setGoal(goal_constraints);

  // plan
  auto plan_solution = planning_components->plan(plan_params);

  if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(*logger_, "Planning fail");
    success = false;
    return success;
  }

  success = true;
  RCLCPP_INFO(*logger_, "Planning success");

  result = plan_solution.trajectory;
  // need time_parametrization
  return success;
}

template <class ActionT>
bool ComputePathActionServerCore<ActionT>::plan(
  const moveit_cpp::MoveItCppPtr & moveit_cpp_ptr,
  planning_scene_monitor::PlanningSceneMonitorPtr psm,
  moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params,
  const moveit::core::LinkModel & link, const Eigen::Isometry3d & offset,
  const Eigen::Isometry3d & target_eigen, const moveit::core::JointModelGroup * jmg,
  const moveit_msgs::msg::Constraints & path_constraints, double goal_position_tolerance,
  double goal_orientation_tolerance, robot_trajectory::RobotTrajectoryPtr & result)
{
  bool success = false;

  auto start_robot_state = moveit_msgs::msg::RobotState();
  moveit::core::robotStateToRobotStateMsg(
    planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState(), start_robot_state);
  RCLCPP_DEBUG(
    *logger_, "Ready for planning: robot start state: %s",
    moveit_msgs::msg::to_yaml(start_robot_state).c_str());

  // Create planning component
  auto planning_components =
    std::make_shared<moveit_cpp::PlanningComponent>(jmg->getName(), moveit_cpp_ptr);

  // set Path Constraints
  planning_components->setPathConstraints(path_constraints);

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = planning_scene_monitor::LockedPlanningSceneRO(psm)->getPlanningFrame();
  target.pose = tf2::toMsg(target_eigen * offset.inverse());
  RCLCPP_DEBUG(
    *logger_, "Prepare for planning, goal:\n target frame: %s\n posestamp: %s",
    link.getName().c_str(), geometry_msgs::msg::to_yaml(target).c_str());

  std::vector<moveit_msgs::msg::Constraints> goal_constraints;
  goal_constraints.push_back(kinematic_constraints::constructGoalConstraints(
    link.getName(), target, goal_position_tolerance, goal_orientation_tolerance));

  // Copy goal constraint into planning component
  planning_components->setGoal(goal_constraints);
  planning_components->setStartStateToCurrentState();
  // plan
  auto plan_solution = planning_components->plan(plan_params);

  if (plan_solution.error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    success = false;
    return success;
  }

  success = true;
  result = plan_solution.trajectory;
  return success;
}

template <class ActionT>
void ComputePathActionServerCore<ActionT>::trajectoryTimeParametrization(
  robot_trajectory::RobotTrajectoryPtr & result,
  moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params)
{
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parametrization;
  time_parametrization.computeTimeStamps(
    *result, plan_params.max_velocity_scaling_factor, plan_params.max_acceleration_scaling_factor);
}

template <class ActionT>
bool ComputePathActionServerCore<ActionT>::computePath(
  const moveit_cpp::MoveItCppPtr & moveit_cpp_ptr,
  moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params, const std::string & group,
  const boost::any & goal, planning_scene_monitor::PlanningSceneMonitorPtr psm,
  double goal_joint_tolerance, double goal_orientation_tolerance, double goal_position_tolerance,
  robot_trajectory::RobotTrajectoryPtr & robot_trajectory,
  const moveit_msgs::msg::Constraints & path_constraints, const std::string & ik_frame_id)
{
  const moveit::core::JointModelGroup * jmg =
    planning_scene_monitor::LockedPlanningSceneRO(psm)->getRobotModel()->getJointModelGroup(group);
  moveit::core::RobotState current_robot_state =
    planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState();
  moveit::core::RobotState target_robot_state = current_robot_state;

  bool success;

  if (!jmg) {
    RCLCPP_ERROR(*logger_, "Could not joint group from %s", group.c_str());
    return false;
  }
  if (goal.empty()) {
    RCLCPP_ERROR(*logger_, "Goal is empty");
    return false;
  }

  moveit_msgs::msg::RobotState current_state_msg;
  moveit::core::robotStateToRobotStateMsg(current_robot_state, current_state_msg);
  RCLCPP_DEBUG(
    *logger_, "Before transferring goal, current robot state: %s",
    moveit_msgs::msg::to_yaml(current_state_msg).c_str());

  if (getJointStateGoal(goal, jmg, target_robot_state)) {
    success = plan(
      moveit_cpp_ptr, psm, plan_params, target_robot_state, jmg, path_constraints,
      goal_joint_tolerance, robot_trajectory);
    return success;
  } else {
    const planning_scene::PlanningScenePtr lscene = [&] {
      planning_scene_monitor::LockedPlanningSceneRO ls(psm);
      return planning_scene::PlanningScene::clone(ls);
    }();

    current_robot_state = lscene->getCurrentState();

    Eigen::Isometry3d target;

    // tip link
    const moveit::core::LinkModel * link;
    // the translation vector from link to global(planning) frame
    Eigen::Isometry3d ik_pose_world;
    if (!getRobotTipForFrame(lscene, jmg, link, ik_pose_world, ik_frame_id)) return false;

    if (!getPoseGoal(goal, lscene, target) && !getPointGoal(goal, ik_pose_world, lscene, target)) {
      RCLCPP_ERROR(*logger_, "Invalid goal type: %s", goal.type().name());
      return false;
    }

    // offset from link to ik_frame
    Eigen::Isometry3d offset =
      lscene->getCurrentState().getGlobalLinkTransform(link).inverse() * ik_pose_world;

    geometry_msgs::msg::Pose end_pose =
      tf2::toMsg(current_robot_state.getGlobalLinkTransform(link));
    RCLCPP_INFO(
      *logger_, "Current pose of %s: %s", link->getName().c_str(),
      geometry_msgs::msg::to_yaml(end_pose).c_str());

    // checkCollision(lscene);
    success = plan(
      moveit_cpp_ptr, psm, plan_params, *link, offset, target, jmg, path_constraints,
      goal_position_tolerance, goal_orientation_tolerance, robot_trajectory);
  }

  if (success) {
    RCLCPP_INFO(*logger_, "Planning succeeded");
    return true;
  } else {
    RCLCPP_ERROR(*logger_, "Fail planning");
    return false;
  }

  // need time parametrization
}

template <class ActionT>
bool ComputePathActionServerCore<ActionT>::getJointStateGoal(
  const boost::any & goal, const moveit::core::JointModelGroup * jmg,
  moveit::core::RobotState & target_state)
{
  try {
    // try named joint pose
    const std::string & named_joint_pose = boost::any_cast<std::string>(goal);
    if (!target_state.setToDefaultValues(jmg, named_joint_pose))
      RCLCPP_ERROR(*logger_, "Unknown joint pose: %s", named_joint_pose.c_str());
    RCLCPP_INFO(*logger_, "Get named_joint_pose: %s", named_joint_pose.c_str());
    target_state.update();
    return true;
  } catch (const boost::bad_any_cast &) {
  }

  try {
    // try RobotState
    const moveit_msgs::msg::RobotState & msg = boost::any_cast<moveit_msgs::msg::RobotState>(goal);
    if (!msg.is_diff) RCLCPP_ERROR(*logger_, "Expecting a diff state.");
    // validate specified joints
    const auto & accepted = jmg->getJointModelNames();
    for (const auto & name : msg.joint_state.name)
      if (std::find(accepted.begin(), accepted.end(), name) == accepted.end())
        RCLCPP_ERROR(
          *logger_, "Joint '%s' is not part of group '%s'", name.c_str(), jmg->getName().c_str());
    for (const auto & name : msg.multi_dof_joint_state.joint_names)
      if (std::find(accepted.begin(), accepted.end(), name) == accepted.end())
        RCLCPP_ERROR(
          *logger_, "Joint '%s' is not part of group '%s'", name.c_str(), jmg->getName().c_str());

    moveit::core::robotStateMsgToRobotState(msg, target_state, false);
    return true;
  } catch (const boost::bad_any_cast &) {
  }

  try {
    // try joint map
    const std::map<std::string, double> & joint_map =
      boost::any_cast<std::map<std::string, double>>(goal);
    const auto & accepted = jmg->getJointModelNames();
    for (const auto & joint : joint_map) {
      if (std::find(accepted.begin(), accepted.end(), joint.first) == accepted.end())
        RCLCPP_ERROR(
          *logger_, "Joint '%s' is not part of group '%s'", joint.first.c_str(),
          jmg->getName().c_str());
      target_state.setVariablePosition(joint.first, joint.second);
    }
    target_state.update();
    return true;
  } catch (const boost::bad_any_cast &) {
  }
  return false;
}

template <class ActionT>
bool ComputePathActionServerCore<ActionT>::getPoseGoal(
  const boost::any & goal, const planning_scene::PlanningSceneConstPtr & scene,
  Eigen::Isometry3d & target)
{
  try {
    const geometry_msgs::msg::PoseStamped & msg =
      boost::any_cast<geometry_msgs::msg::PoseStamped>(goal);
    tf2::fromMsg(msg.pose, target);

    // transform target into global(planning) frame
    target = scene->getFrameTransform(msg.header.frame_id) * target;
    geometry_msgs::msg::Pose new_msg;
    new_msg = tf2::toMsg(target);
    RCLCPP_INFO(
      *logger_, "Get pose goal: \n origin: %s \n transfer to planning frame %s: %s",
      geometry_msgs::msg::to_yaml(msg).c_str(), scene->getPlanningFrame().c_str(),
      geometry_msgs::msg::to_yaml(new_msg).c_str());
  } catch (const boost::bad_any_cast &) {
    return false;
  }
  return true;
}

template <class ActionT>
bool ComputePathActionServerCore<ActionT>::getPointGoal(
  const boost::any & goal, const Eigen::Isometry3d & ik_pose,
  const planning_scene::PlanningSceneConstPtr & scene, Eigen::Isometry3d & target_eigen)
{
  try {
    const geometry_msgs::msg::PointStamped & target =
      boost::any_cast<geometry_msgs::msg::PointStamped>(goal);
    Eigen::Vector3d target_point;
    tf2::fromMsg(target.point, target_point);
    // transform target into global(planning) frame
    target_point = scene->getFrameTransform(target.header.frame_id) * target_point;

    // retain link orientation
    target_eigen = ik_pose;
    target_eigen.translation() = target_point;
    RCLCPP_INFO(
      *logger_, "Get point goal: \n origin: %s \n transfer to planning frame [%s]: %s",
      geometry_msgs::msg::to_yaml(target).c_str(), scene->getPlanningFrame().c_str(),
      geometry_msgs::msg::to_yaml(tf2::toMsg(target_eigen)).c_str());
  } catch (const boost::bad_any_cast &) {
    return false;
  }
  return true;
}

}  // namespace moveit_skills
