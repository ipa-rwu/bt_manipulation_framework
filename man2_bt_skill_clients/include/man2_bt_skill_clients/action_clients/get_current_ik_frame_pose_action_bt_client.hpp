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

#ifndef MAN2_BT_SKILL_CLIENTS__GET_CURRENT_IK_FRAME_POSE_ACTION_CLIENT_HPP_
#define MAN2_BT_SKILL_CLIENTS__GET_CURRENT_IK_FRAME_POSE_ACTION_CLIENT_HPP_

#include "man2_behavior_tree/bt_action_client_node.hpp"
#include "man2_behavior_tree/bt_conversions.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_skills/action/get_current_ik_frame_pose.hpp"

namespace man2_bt_skill_clients
{
class GetCurrentIKFramePoseActionClient
: public ros2_behavior_tree::BtActionClientNode<moveit_skills::action::GetCurrentIKFramePose>
{
public:
  using Action = moveit_skills::action::GetCurrentIKFramePose;
  using ActionResult = moveit_skills::action::GetCurrentIKFramePose::Result;

  GetCurrentIKFramePoseActionClient(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : ros2_behavior_tree::BtActionClientNode<Action>(xml_tag_name, "get_current_ik_frame_pose", conf)
  {
  }

  void on_tick() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("group_name", "The target move group name"),
      BT::InputPort<std::string>("ik_frame", "The target IK frame name"),
      BT::OutputPort<geometry_msgs::msg::Pose>("pose", "The pose of IK frame"),
      BT::OutputPort<ActionResult::_error_code_id_type>("error_code_id", "Action error code ID"),
    });
  }
};
}  // namespace man2_bt_skill_clients

#endif  // MAN2_BT_SKILL_CLIENTS__GET_CURRENT_IK_FRAME_POSE_ACTION_CLIENT_HPP_
