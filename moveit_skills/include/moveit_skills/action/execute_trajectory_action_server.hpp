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
#include "moveit_skills/action/execute_trajectory.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace moveit_skills
{
class ExecuteTrajectoryActionServer
{
public:
  using ActionT = moveit_skills::action::ExecuteTrajectory;
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  ExecuteTrajectoryActionServer(
    const nav2_util::LifecycleNode::WeakPtr & parent, const std::string & action_name,
    moveit_cpp::MoveItCppPtr & moveit_cpp_ptr,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~ExecuteTrajectoryActionServer();

  void execution();

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

protected:
  /**
   * @brief Wrapper function to get current goal
   * @return Shared pointer to current action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getCurrentGoal() const
  {
    return action_server_->get_current_goal();
  }

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

  std::string action_name_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr_;
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Node::SharedPtr node_;

  robot_trajectory::RobotTrajectoryPtr traj_;
};

}  // namespace moveit_skills

#endif  // MOVEIT_SKILLS__EXECUTE_TRAJECTORY_ACTION_SERVER_HPP_
