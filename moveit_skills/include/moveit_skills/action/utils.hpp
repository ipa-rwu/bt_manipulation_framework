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

#ifndef MOVEIT_SKILLS__UTILS_HPP_
#define MOVEIT_SKILLS__UTILS_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "nav2_util/lifecycle_node.hpp"

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

bool getRobotTipForFrame(
  const planning_scene::PlanningSceneConstPtr & scene, const moveit::core::JointModelGroup * jmg,
  const moveit::core::LinkModel *& robot_link, Eigen::Isometry3d & tip_in_global_frame,
  const std::string ik_frame_id)
{
  auto get_tip = [&jmg]() -> const moveit::core::LinkModel * {
    // determine IK frame from group
    std::vector<const moveit::core::LinkModel *> tips;
    jmg->getEndEffectorTips(tips);
    if (tips.size() != 1) {
      return nullptr;
    }
    return tips[0];
  };

  if (ik_frame_id.empty()) {
    robot_link = get_tip();
    if (!robot_link) {
      return false;
    }
    // transfer of tip (tip pose) in planning frame
    tip_in_global_frame = scene->getCurrentState().getGlobalLinkTransform(robot_link);
  } else {
    robot_link = nullptr;
    bool found = false;
    // if found, get transfer ik_frame_id (ik_frame pose) in planning frame
    auto ref_frame = scene->getCurrentState().getFrameInfo(ik_frame_id, robot_link, found);
    if (!found && !ik_frame_id.empty()) {
      return false;
    }
    if (!robot_link) robot_link = get_tip();
    if (!robot_link) {
      return false;
    }

    if (found) {  // use robot link's frame as reference by default
      tip_in_global_frame = ref_frame;
    } else {
      tip_in_global_frame = scene->getCurrentState().getGlobalLinkTransform(robot_link);
    }
  }

  return true;
}

}  // namespace moveit_skills

#endif  // MOVEIT_SKILLS__UTILS_HPP_
