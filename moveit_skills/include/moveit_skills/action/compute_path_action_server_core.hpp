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
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace moveit_skills
{
class RosParameters
{
public:
  std::string planner_id;
  double planning_time;
  int planning_attempts;
  double goal_position_tolerance;     // 0.1 mm
  double goal_orientation_tolerance;  // ~0.1 deg
  double goal_joint_tolerance;
  double step_size;
  double jump_threshold;
  double max_velocity_scaling_factor;
  double max_acceleration_scaling_factor;
  double min_fraction;
  std::string planning_pipeline;
  std::string get_planning_scene_service_name;
  std::string PILZ_PLANNING_PIPELINE = "pilz_industrial_motion_planner";

  std::string ns = "";

  RosParameters(const nav2_util::LifecycleNode::SharedPtr & node)
  {
    if (!node->has_parameter(ns + "planning_pipeline")) {
      node->declare_parameter(ns + "planning_pipeline", std::string(""));
    }
    if (!node->has_parameter(ns + "planner_id")) {
      node->declare_parameter(ns + "planner_id", std::string(""));
    }
    if (!node->has_parameter(ns + "planning_time")) {
      node->declare_parameter(ns + "planning_time", 5.0);
    }
    if (!node->has_parameter(ns + "planning_attempts")) {
      node->declare_parameter(ns + "planning_attempts", 5);
    }
    if (!node->has_parameter(ns + "goal_joint_tolerance")) {
      node->declare_parameter(ns + "goal_joint_tolerance", 1e-4);
    }
    if (!node->has_parameter(ns + "goal_position_tolerance")) {
      node->declare_parameter(ns + "goal_position_tolerance", 1e-4);
    }
    if (!node->has_parameter(ns + "goal_orientation_tolerance")) {
      node->declare_parameter(ns + "goal_orientation_tolerance", 1e-3);
    }
    if (!node->has_parameter(ns + "step_size")) {
      node->declare_parameter(ns + "step_size", 0.005);
    }
    if (!node->has_parameter(ns + "jump_threshold")) {
      node->declare_parameter(ns + "jump_threshold", 0.0);
    }
    if (!node->has_parameter(ns + "max_velocity_scaling_factor")) {
      node->declare_parameter(ns + "max_velocity_scaling_factor", 0.5);
    }
    if (!node->has_parameter(ns + "max_velocity_scaling_factor")) {
      node->declare_parameter(ns + "max_velocity_scaling_factor", 0.5);
    }
    if (!node->has_parameter(ns + "max_acceleration_scaling_factor")) {
      node->declare_parameter(ns + "max_acceleration_scaling_factor", 0.5);
    }
    if (!node->has_parameter(ns + "min_fraction")) {
      node->declare_parameter(ns + "min_fraction", 0.7);
    }

    if (!node->has_parameter(ns + "get_planning_scene_service_name")) {
      node->declare_parameter(
        ns + "get_planning_scene_service_name",
        std::string("compute_path_moveitcpp_skill/get_planning_scene"));
    }
  }

  void loadRosParameters(const nav2_util::LifecycleNode::SharedPtr & node)
  {
    node->get_parameter(ns + "planner_id", planner_id);
    node->get_parameter(ns + "planning_time", planning_time);
    node->get_parameter(ns + "planning_attempts", planning_attempts);
    node->get_parameter(ns + "goal_joint_tolerance", goal_position_tolerance);
    node->get_parameter(ns + "goal_position_tolerance", goal_position_tolerance);
    node->get_parameter(ns + "goal_orientation_tolerance", goal_orientation_tolerance);
    node->get_parameter(ns + "step_size", step_size);
    node->get_parameter(ns + "jump_threshold", jump_threshold);
    node->get_parameter(ns + "max_velocity_scaling_factor", max_velocity_scaling_factor);
    node->get_parameter(ns + "max_acceleration_scaling_factor", max_acceleration_scaling_factor);
    node->get_parameter(ns + "min_fraction", min_fraction);
    node->get_parameter(ns + "planning_pipeline", planning_pipeline);
    node->get_parameter(ns + "get_planning_scene_service_name", get_planning_scene_service_name);
  }
};

template <class ActionT>
class ComputePathActionServerCore
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  ComputePathActionServerCore() {}

  ~ComputePathActionServerCore() {}

  /**
   * @brief Wrapper function to get current goal
   * @return Shared pointer to current action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    return action_server_->get_current_goal();
  }

  /**
   * @brief Wrapper function to get pending goal
   * @return Shared pointer to pending action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const
  {
    return action_server_->get_pending_goal();
  }

  /**
 * @brief Activate action server
 *
 */
  void activate() { action_server_->activate(); }

  /**
 * @brief Deactivate action server
 *
 */
  void deactivate() { action_server_->deactivate(); }

  /**
   * @brief Wrapper function to send succeeded goal
   */
  void terminatePendingGoal() { action_server_->terminate_pending_goal(); }

  void sendSucceededResult(const std::shared_ptr<typename ActionT::Result> result)
  {
    action_server_->succeeded_current(result);
  }

  /**
   * @brief Wrapper function to terminate current goal by sending failed result
   */
  void sendFaildResult(const std::shared_ptr<typename ActionT::Result> result)
  {
    action_server_->terminate_current(result);
  }

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

  bool getRobotTipForFrame(
    const planning_scene::PlanningSceneConstPtr & scene, const moveit::core::JointModelGroup * jmg,
    const moveit::core::LinkModel *& robot_link, Eigen::Isometry3d & tip_in_global_frame,
    const std::string ik_frame_id);

  bool getPoseGoal(
    const boost::any & goal, const planning_scene::PlanningSceneConstPtr & scene,
    Eigen::Isometry3d & target);

  bool getPointGoal(
    const boost::any & goal, const Eigen::Isometry3d & ik_pose,
    const planning_scene::PlanningSceneConstPtr & scene, Eigen::Isometry3d & target_eigen);

  std::shared_ptr<RosParameters> parameters_;

protected:
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<moveit_cpp::PlanningComponent::PlanRequestParameters> plan_params_;
  bool set_planner_called_ = false;
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Node::SharedPtr node_;
};
}  // namespace moveit_skills

#include "moveit_skills/action/compute_path_action_server_core_impl.hpp"

#endif  //MOVEIT_SKILLS__COMPUTE_PATH_UTIL_HPP_
