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

#include "man2_bt_skill_clients/action_clients/execute_trajectory_action_bt_client.hpp"

namespace man2_bt_skill_clients
{
void ExecuteTrajectoryActionClient::on_tick()
{
  {
    auto res = getInput<std::string>("target_group");
    if (!res) {
      throw BT::RuntimeError("error reading port [target_group]:", res.error());
    } else {
      goal_.target_group = res.value();
    }
  }

  auto res = getInput<moveit_msgs::msg::RobotTrajectory>("trajectory");
  if (!res) {
    throw BT::RuntimeError("error reading port [trajectory]:", res.error());
  } else {
    goal_.trajectory = res.value();
  }

  RCLCPP_DEBUG(
    *logger_, "target_group: %s, trajectory:%s", goal_.target_group.c_str(),
    moveit_msgs::msg::to_yaml(goal_.trajectory).c_str());
}

BT::NodeStatus ExecuteTrajectoryActionClient::on_success()
{
  // Set empty error code, action was successful
  setOutput("error_code_id", result_.result->error_code_id);
  if (result_.result->error_code_id == ActionResult::NONE) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace man2_bt_skill_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(
    BT::CreateManifest<man2_bt_skill_clients::ExecuteTrajectoryActionClient>(
      "ExecuteTrajectory", man2_bt_skill_clients::ExecuteTrajectoryActionClient::providedPorts()),
    BT::CreateBuilder<man2_bt_skill_clients::ExecuteTrajectoryActionClient>());
}
