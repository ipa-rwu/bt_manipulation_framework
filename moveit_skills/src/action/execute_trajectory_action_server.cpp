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

#include "moveit_skills/action/execute_trajectory_action_server.hpp"

namespace moveit_skills
{
ExecuteTrajectoryActionServer::ExecuteTrajectoryActionServer(
  const nav2_util::LifecycleNode::WeakPtr & parent, const std::string & action_name,
  moveit_cpp::MoveItCppPtr & moveit_cpp_ptr, const rclcpp::NodeOptions & /*options*/)
: action_name_(action_name), moveit_cpp_ptr_(moveit_cpp_ptr)
{
  auto node = parent.lock();
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(node->get_name() + action_name_));

  action_server_ = std::make_unique<ActionServer>(
    node, action_name_, std::bind(&ExecuteTrajectoryActionServer::execution, this), nullptr,
    std::chrono::milliseconds(500), true);
}

ExecuteTrajectoryActionServer::~ExecuteTrajectoryActionServer() {}

void ExecuteTrajectoryActionServer::execution()
{
  auto goal = getCurrentGoal();
  auto result = std::make_shared<ActionT::Result>();

  traj_ = std::make_shared<robot_trajectory::RobotTrajectory>(
    moveit_cpp_ptr_->getRobotModel(), goal->target_group);
  //   robot_trajectory::RobotTrajectory rt(moveit_cpp_ptr_->getRobotModel(), goal->target_group);
  traj_->setRobotTrajectoryMsg(*moveit_cpp_ptr_->getCurrentState(), goal->trajectory);

  if (moveit_cpp_ptr_->execute(goal->target_group, traj_)) {
    result->error_code_id = result->NONE;
    sendSucceededResult(result);
  }
  result->error_code_id = result->EXECUTION_FAIL;
  sendFaildResult(result);
}

}  // namespace moveit_skills
