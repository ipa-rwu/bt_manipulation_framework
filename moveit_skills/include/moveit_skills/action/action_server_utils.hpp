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

#ifndef MOVEIT_SKILLS__ACTION_SERVER_UTILS_HPP_
#define MOVEIT_SKILLS__ACTION_SERVER_UTILS_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

namespace moveit_skills
{
template <class ActionT>
class ActionServerUtils
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  ActionServerUtils() {}

  ~ActionServerUtils() {}

  virtual void execution() {}

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
   * @brief Wrapper function to get pending goal
   * @return Shared pointer to pending action goal
   */
  const std::shared_ptr<const typename ActionT::Goal> getPendingGoal() const
  {
    return action_server_->get_pending_goal();
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
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<rclcpp::Logger> logger_;
};

}  // namespace moveit_skills

#endif  // MOVEIT_SKILLS__ACTION_SERVER_UTILS_HPP_
