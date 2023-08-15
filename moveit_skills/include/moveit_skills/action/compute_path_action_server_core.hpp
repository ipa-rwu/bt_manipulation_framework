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

#ifndef MOVEIT_SKILLS__COMPUTE_PATH_UTIL_HPP_
#define MOVEIT_SKILLS__COMPUTE_PATH_UTIL_HPP_

#include <boost/any.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "moveit/kinematic_constraints/utils.h"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/moveit_cpp/planning_component.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit_skills/action/utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace moveit_skills
{
template <class ActionT>
class ComputePathActionServerCore : public ActionServerUtils<ActionT>

{
public:
  ComputePathActionServerCore() : ActionServerUtils<ActionT>() {}

  ~ComputePathActionServerCore() {}

  void initPlanComponentParameters(
    moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params,
    const RosParameters & parameters);

  void setPlanner(
    moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params,
    const std::string & planning_pipeline, const std::string & planner_id);

  bool plan(
    const moveit_cpp::MoveItCppPtr & moveit_cpp_ptr,
    planning_scene_monitor::PlanningSceneMonitorPtr psm,
    moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params,
    const moveit::core::RobotState & target_robot_state, const moveit::core::JointModelGroup * jmg,
    const moveit_msgs::msg::Constraints & path_constraints, double goal_joint_tolerance,
    robot_trajectory::RobotTrajectoryPtr & result);

  bool plan(
    const moveit_cpp::MoveItCppPtr & moveit_cpp_ptr,
    planning_scene_monitor::PlanningSceneMonitorPtr psm,
    moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params,
    const moveit::core::LinkModel & link, const Eigen::Isometry3d & offset,
    const Eigen::Isometry3d & target_eigen, const moveit::core::JointModelGroup * jmg,
    const moveit_msgs::msg::Constraints & path_constraints, double goal_position_tolerance,
    double goal_orientation_tolerance, robot_trajectory::RobotTrajectoryPtr & result);

  /**
   * @brief Check goal type, transfer into moveit::core::RobotState
   *
   * @param goal can be std::string named joint state; moveit_msgs::msg::RobotState, joint map
   * @param jmg moveit::core::JointModelGroup
   * @param target_state result: moveit::core::RobotState
   * @return true
   * @return false
   */
  bool getJointStateGoal(
    const boost::any & goal, const moveit::core::JointModelGroup * jmg,
    moveit::core::RobotState & state);

  void trajectoryTimeParametrization(
    robot_trajectory::RobotTrajectoryPtr & result,
    moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params);

  bool computePath(
    const moveit_cpp::MoveItCppPtr & moveit_cpp_ptr,
    moveit_cpp::PlanningComponent::PlanRequestParameters & plan_params, const std::string & group,
    const boost::any & goal, planning_scene_monitor::PlanningSceneMonitorPtr psm,
    double goal_joint_tolerance, double goal_orientation_tolerance, double goal_position_tolerance,
    robot_trajectory::RobotTrajectoryPtr & robot_trajectory,
    const moveit_msgs::msg::Constraints & path_constraints = moveit_msgs::msg::Constraints(),
    const std::string & ik_frame_id = "");

  bool getPoseGoal(
    const boost::any & goal, const planning_scene::PlanningSceneConstPtr & scene,
    Eigen::Isometry3d & target);

  bool getPointGoal(
    const boost::any & goal, const Eigen::Isometry3d & ik_pose,
    const planning_scene::PlanningSceneConstPtr & scene, Eigen::Isometry3d & target_eigen);

  std::shared_ptr<RosParameters> parameters_;

protected:
  std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters> plan_params_;
  std::shared_ptr<rclcpp::Logger> logger_;
};
}  // namespace moveit_skills

#include "moveit_skills/action/compute_path_action_server_core_impl.hpp"

#endif  //MOVEIT_SKILLS__COMPUTE_PATH_UTIL_HPP_
