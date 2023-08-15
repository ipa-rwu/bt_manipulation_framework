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

#include "man2_bt_skill_clients/action_clients/compute_path_to_pose_action_bt_client.hpp"

namespace man2_bt_skill_clients
{
void ComputePathToPoseActionClient::on_tick()
{
  {
    auto res = getInput<std::string>("target_group");
    if (!res) {
      throw BT::RuntimeError("error reading port [target_group]:", res.error());
    } else {
      goal_.target_group = res.value();
    }
  }

  auto res = getInput<geometry_msgs::msg::PoseStamped>("target_pose");
  if (!res) {
    throw BT::RuntimeError("error reading port [target_pose]:", res.error());
  } else {
    goal_.target_pose = res.value();
  }

  {
    auto res = getInput<moveit_msgs::msg::Constraints>("path_constraints");
    if (res) {
      goal_.config.path_constraints = res.value();
    }
  }

  {
    auto res = getInput<std::string>("ik_frame");
    if (res) {
      goal_.config.ik_frame = res.value();
    }
  }

  {
    auto res = getInput<std::string>("planning_pipeline");
    if (res) {
      goal_.config.planning_pipeline = res.value();
    }
  }

  {
    auto res = getInput<std::string>("planner_id");
    if (res) {
      goal_.config.planner_id = res.value();
    }
  }

  RCLCPP_INFO(
    *logger_, "target_group: %s, target_pose:%s, planning_pipeline: %s, planner_id: %s\n",
    goal_.target_group.c_str(), geometry_msgs::msg::to_yaml(goal_.target_pose).c_str(),
    goal_.config.planning_pipeline.c_str(), goal_.config.planner_id.c_str());
}

BT::NodeStatus ComputePathToPoseActionClient::on_success()
{
  setOutput("trajectory", result_.result->plan_result.trajectory);
  // Set empty error code, action was successful
  setOutput("error_code_id", result_.result->plan_result.error_code_id);
  if (result_.result->plan_result.error_code_id == ComputePathResult::NONE)
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}

}  // namespace man2_bt_skill_clients

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(
    BT::CreateManifest<man2_bt_skill_clients::ComputePathToPoseActionClient>(
      "ComputePathToPose", man2_bt_skill_clients::ComputePathToPoseActionClient::providedPorts()),
    BT::CreateBuilder<man2_bt_skill_clients::ComputePathToPoseActionClient>());
}
