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

#include "man2_behavior_tree/util_plugins/print_value.hpp"

namespace util_plugins
{
PrintValue::PrintValue(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus PrintValue::tick()
{
  std::string msg;
  if (getInput("message", msg)) {
    std::cout << "PrintValue: " << msg << std::endl;
    return BT::NodeStatus::SUCCESS;
  } else {
    std::cout << "PrintValue FAILED " << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList PrintValue::providedPorts() { return {BT::InputPort<std::string>("message")}; }

}  // namespace util_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(
    BT::CreateManifest<util_plugins::PrintValue>(
      "PrintValue", util_plugins::PrintValue::providedPorts()),
    BT::CreateBuilder<util_plugins::PrintValue>());
}
