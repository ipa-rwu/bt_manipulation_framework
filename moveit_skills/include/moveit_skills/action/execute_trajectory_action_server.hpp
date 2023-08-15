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

#ifndef MOVEIT_SKILLS__EXECUTE_TRAJECTORY_ACTION_SERVER_HPP_
#define MOVEIT_SKILLS__EXECUTE_TRAJECTORY_ACTION_SERVER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit_skills/action/action_server_utils.hpp"
#include "moveit_skills/action/execute_trajectory.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace moveit_skills
{
class ExecuteTrajectoryActionServer
: public ActionServerUtils<moveit_skills::action::ExecuteTrajectory>
{
public:
  using ActionT = moveit_skills::action::ExecuteTrajectory;

  ExecuteTrajectoryActionServer(
    const nav2_util::LifecycleNode::WeakPtr & parent, const std::string & action_name,
    moveit_cpp::MoveItCppPtr & moveit_cpp_ptr,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ExecuteTrajectoryActionServer();

  void execution() override;

protected:
  std::string action_name_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
  std::shared_ptr<rclcpp::Logger> logger_;

  robot_trajectory::RobotTrajectoryPtr traj_;
};

}  // namespace moveit_skills

#endif  // MOVEIT_SKILLS__EXECUTE_TRAJECTORY_ACTION_SERVER_HPP_
