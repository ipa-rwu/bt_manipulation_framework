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

#ifndef MOVEIT_SKILLS__COMPUTE_PATH_TO_POINT_ACTION_SERVER_HPP_
#define MOVEIT_SKILLS__COMPUTE_PATH_TO_POINT_ACTION_SERVER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "moveit_skills/action/compute_path_to_point.hpp"
#include "moveit_skills/compute_path_action_server_core.hpp"

namespace moveit_skills
{
class ComputePathToPointActionServer
: public moveit_skills::ComputePathActionServerCore<moveit_skills::action::ComputePathToPoint>
{
public:
  using ActionT = moveit_skills::action::ComputePathToPoint;

  ComputePathToPointActionServer(
    const nav2_util::LifecycleNode::WeakPtr & parent, const std::string & action_name,
    moveit_cpp::MoveItCppPtr & moveit_cpp_ptr, planning_scene_monitor::PlanningSceneMonitorPtr psm,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ComputePathToPointActionServer();

  void execution();

protected:
  void initial();

  std::string action_name_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
};

}  // namespace moveit_skills

#endif  // MOVEIT_SKILLS__COMPUTE_PATH_TO_POINT_ACTION_SERVER_HPP_
