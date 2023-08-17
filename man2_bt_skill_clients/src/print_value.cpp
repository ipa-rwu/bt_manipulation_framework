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

#include "man2_bt_skill_clients/print_value.hpp"

namespace man2_bt_skill_clients
{
PrintValue::PrintValue(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus PrintValue::tick()
{
  {
    auto res = getInput<moveit_msgs::msg::RobotTrajectory>("msg_trajectory");
    if (res) {
      std::cout << "PrintValue in [RobotTrajectory] type: "
                << moveit_msgs::msg::to_yaml(res.value()) << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
  }
  {
    auto res = getInput<geometry_msgs::msg::PoseStamped>("msg_posestamped");
    if (res) {
      std::cout << "PrintValue in [PoseStamped] type: " << geometry_msgs::msg::to_yaml(res.value())
                << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
  }
  {
    auto res = getInput<std::string>("msg_string");
    if (res) {
      std::cout << "PrintValue in [string] type: " << res.value() << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
  }
  {
    auto res = getInput<geometry_msgs::msg::Pose>("msg_pose");
    if (res) {
      std::cout << "PrintValue in [Pose] type: " << geometry_msgs::msg::to_yaml(res.value())
                << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::FAILURE;
}

}  // namespace man2_bt_skill_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(
    BT::CreateManifest<man2_bt_skill_clients::PrintValue>(
      "PrintValue", man2_bt_skill_clients::PrintValue::providedPorts()),
    BT::CreateBuilder<man2_bt_skill_clients::PrintValue>());
}
