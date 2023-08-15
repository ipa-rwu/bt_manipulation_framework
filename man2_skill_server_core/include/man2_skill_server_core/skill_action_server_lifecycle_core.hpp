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

#ifndef MAN2_SKILL_SERVER_CORE__SKILL_ACTION_SERVER_LIFECYCLE_CORE_HPP_
#define MAN2_SKILL_SERVER_CORE__SKILL_ACTION_SERVER_LIFECYCLE_CORE_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_skill_server_core
{
/**
 * @class SkillServerCore
 * @brief Abstract interface for behaviors to adhere to with pluginlib
 */
template <class ActionT>
class SkillActionServerLifecycleCore : public nav2_util::LifecycleNode
{
public:
  using ActionServer = nav2_util::SimpleActionServer<ActionT>;

  /**
 * @brief Construct a new Skill Action Server Lifecycle Core object
 *
 * @param action_name action name of a skill; it is as same as ros node name
 * @param options rosnode option setting
 */
  explicit SkillActionServerLifecycleCore(
    const std::string & node_name, const std::string & action_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : nav2_util::LifecycleNode(node_name, "", rclcpp::NodeOptions()), action_name_(action_name)
  {
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~SkillActionServerLifecycleCore() = default;

  /**
 * @brief During lifecycle node configuration time, e.g. initial action server
 *
 * @return nav2_util::CallbackReturn
 */
  virtual nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    // Create the action server that perform some functions
    auto node = shared_from_this();
    action_server_ = std::make_unique<ActionServer>(
      node, action_name_, std::bind(&SkillActionServerLifecycleCore::execution, this), nullptr,
      std::chrono::milliseconds(500), true);
    return nav2_util::CallbackReturn::SUCCESS;
  }

  /**
 * @brief Callback function where a skill action server completes main work. Action should have its own exceptions
 * Get current goal: action_server_->get_current_goal();
 * Process goal, give result
 * If success result: action_server_->succeeded_current(result);
 * If fail: action_server_->terminate_current(result);
 */
  virtual void execution() {}

  /**
 * @brief  When activate lifecycle node
 * active action server
 * if successes, create bond connection to lifecycle manager
 *
 * @return nav2_util::CallbackReturn
 */
  virtual nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(*logger_, "Activating...");
    action_server_->activate();
    if (action_server_->is_server_active()) {  // create bond connection
      createBond();

      return nav2_util::CallbackReturn::SUCCESS;
    }
    return nav2_util::CallbackReturn::FAILURE;
  }

  /**
 * @brief When deactivate lifecycle node
 * deactivate action server
 * destroy bond connection to lifecycle manager
 *
 * @return nav2_util::CallbackReturn
 */
  virtual nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(*logger_, "Deactivating...");
    action_server_->deactivate();
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  /**
 * @brief When cleaning up lifecycle node
 * reset action_server_ pointer
 *
 * @return nav2_util::CallbackReturn
 */
  virtual nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(*logger_, "Cleaning up...");
    action_server_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  /**
 * @brief When shutting down lifecycle node
 *
 * @return nav2_util::CallbackReturn
 */
  virtual nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(*logger_, "Shutting down...");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  const std::shared_ptr<const typename ActionT::Goal> acceptPendingGoal()
  {
    return action_server_->accept_pending_goal();
  }

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

protected:
  std::shared_ptr<ActionServer> action_server_;

  std::string action_name_;

  std::shared_ptr<rclcpp::Logger> logger_;
};

}  // namespace ros2_skill_server_core

#endif  // MAN2_SKILL_SERVER_CORE__SKILL_ACTION_SERVER_LIFECYCLE_CORE_HPP_
