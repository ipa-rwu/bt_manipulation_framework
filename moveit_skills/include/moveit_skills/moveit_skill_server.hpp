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

#ifndef MOVEIT_SKILLS__MOVEIT_SkILL_SERVER_HPP_
#define MOVEIT_SKILLS__MOVEIT_SkILL_SERVER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit_skills/action/compute_path_to_point_action_server.hpp"
#include "moveit_skills/action/compute_path_to_pose_action_server.hpp"
#include "moveit_skills/action/compute_path_to_state_action_server.hpp"
#include "moveit_skills/action/execute_trajectory_action_server.hpp"
#include "moveit_skills/action/get_current_ik_frame_pose_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace moveit_skills
{
class MoveitSkillServer : public nav2_util::LifecycleNode
{
public:
  MoveitSkillServer();

  ~MoveitSkillServer();

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/);

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/);

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/);

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/);

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/);

protected:
  rclcpp::Node::SharedPtr client_node_;

  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  std::shared_ptr<rclcpp::Logger> logger_;

  std::unique_ptr<ComputePathToStateActionServer> compute_path_to_state_action_;
  std::unique_ptr<ComputePathToPointActionServer> compute_path_to_point_action_;
  std::unique_ptr<ComputePathToPoseActionServer> compute_path_to_pose_action_;
  std::unique_ptr<GetCurrentIKFramePoseActionServer> get_current_ik_frame_pose_action_;

  std::unique_ptr<ExecuteTrajectoryActionServer> execute_trajectory_action_;
};
}  // namespace moveit_skills

#endif  // MOVEIT_SKILLS__MOVEIT_SkILL_SERVER_HPP_
