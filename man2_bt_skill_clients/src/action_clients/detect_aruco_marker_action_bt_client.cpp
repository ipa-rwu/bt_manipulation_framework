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

#include "man2_bt_skill_clients/action_clients/detect_aruco_marker_action_bt_client.hpp"

namespace man2_bt_skill_clients
{
void DetectArucoMarkerActionClient::on_tick()
{
  {
    auto res = getInput<int>("marker_id");
    if (!res) {
      throw BT::RuntimeError("error reading port [marker_id]:", res.error());
    } else {
      goal_.marker_id = res.value();
    }
  }

  {
    auto res = getInput<std::string>("sub_topic_name");
    if (res) {
      goal_.sub_topic_name = res.value();
    }
  }

  {
    auto res = getInput<int>("timeout");
    if (res) {
      goal_.timeout = res.value();
    }
  }

  {
    auto res = getInput<int>("required_pose_num");
    if (res) {
      goal_.required_pose_num = res.value();
    }
  }

  RCLCPP_INFO(
    *logger_, "marker_id: %d, timeout:%d, sub_topic_name: %s, required_pose_num:%d\n",
    goal_.marker_id, goal_.timeout, goal_.sub_topic_name.c_str(), goal_.required_pose_num);
}

BT::NodeStatus DetectArucoMarkerActionClient::on_success()
{
  setOutput("marker_pose", result_.result->pose);
  // Set empty error code, action was successful
  setOutput("error_code_id", ActionResult::NONE);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace man2_bt_skill_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(
    BT::CreateManifest<man2_bt_skill_clients::DetectArucoMarkerActionClient>(
      "DetectArucoMarkerClient",
      man2_bt_skill_clients::DetectArucoMarkerActionClient::providedPorts()),
    BT::CreateBuilder<man2_bt_skill_clients::DetectArucoMarkerActionClient>());
}
