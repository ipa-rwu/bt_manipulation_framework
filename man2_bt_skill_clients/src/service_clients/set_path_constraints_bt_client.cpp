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

#include "man2_bt_skill_clients/service_clients/set_path_constraints_bt_client.hpp"

namespace man2_bt_skill_clients
{
void SetPathConstrainsServiceClient::on_tick()
{
  {
    auto res = getInput<std::string>("file_path");
    if (!res) {
      throw BT::RuntimeError("error reading port [file_path]:", res.error());
    } else {
      request_->file_path = res.value();
    }
  }

  RCLCPP_INFO(*logger_, "file_path: %s", request_->file_path.c_str());
}

BT::NodeStatus SetPathConstrainsServiceClient::on_completion(
  std::shared_ptr<Service::Response> response)
{
  if (response->error_code_id == response->NONE) {
    setOutput("error_code_id", response->NONE);
    setOutput("path_constraints", response->path_constraints);
    return BT::NodeStatus::SUCCESS;

  } else {
    setOutput("error_code_id", response->error_code_id);
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace man2_bt_skill_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(
    BT::CreateManifest<man2_bt_skill_clients::SetPathConstrainsServiceClient>(
      "SetPathConstrains", man2_bt_skill_clients::SetPathConstrainsServiceClient::providedPorts()),
    BT::CreateBuilder<man2_bt_skill_clients::SetPathConstrainsServiceClient>());
}
