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

#include "man2_bt_skill_clients/action_clients/get_current_ik_frame_pose_action_bt_client.hpp"

namespace man2_bt_skill_clients
{
void GetCurrentIKFramePoseActionClient::on_tick()
{
  {
    auto res = getInput<std::string>("group_name");
    if (!res) {
      throw BT::RuntimeError("error reading port [group_name]:", res.error());
    } else {
      goal_.group_name = res.value();
    }
  }

  {
    auto res = getInput<std::string>("ik_frame");
    if (!res) {
      throw BT::RuntimeError("error reading port [ik_frame]:", res.error());
    } else {
      goal_.ik_frame = res.value();
    }
  }
}

BT::NodeStatus GetCurrentIKFramePoseActionClient::on_success()
{
  // Set empty error code, action was successful
  setOutput("error_code_id", result_.result->error_code_id);
  if (result_.result->error_code_id == ActionResult::NONE) {
    setOutput("pose", result_.result->pose);
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
    BT::CreateManifest<man2_bt_skill_clients::GetCurrentIKFramePoseActionClient>(
      "GetCurrentIKFramePose",
      man2_bt_skill_clients::GetCurrentIKFramePoseActionClient::providedPorts()),
    BT::CreateBuilder<man2_bt_skill_clients::GetCurrentIKFramePoseActionClient>());
}
