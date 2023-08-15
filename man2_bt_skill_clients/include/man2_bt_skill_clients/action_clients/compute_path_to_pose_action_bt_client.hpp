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

#ifndef MAN2_BT_SKILL_CLIENTS__COMPUTE_PATH_TO_POSE_ACTION_CLIENT_HPP_
#define MAN2_BT_SKILL_CLIENTS__COMPUTE_PATH_TO_POSE_ACTION_CLIENT_HPP_

#include "man2_behavior_tree/bt_action_client_node.hpp"
#include "man2_behavior_tree/bt_conversions.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_skills/action/compute_path_to_pose.hpp"
#include "moveit_skills/msg/compute_path_result.hpp"

namespace man2_bt_skill_clients
{
class ComputePathToPoseActionClient
: public ros2_behavior_tree::BtActionClientNode<moveit_skills::action::ComputePathToPose>
{
public:
  using Action = moveit_skills::action::ComputePathToPose;
  using ActionResult = moveit_skills::action::ComputePathToPose::Result;
  using ComputePathResult = moveit_skills::msg::ComputePathResult;

  ComputePathToPoseActionClient(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : ros2_behavior_tree::BtActionClientNode<Action>(xml_tag_name, "compute_path_to_pose", conf)
  {
  }

  void on_tick() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("target_group", "The target move group"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "target_pose", "The pose stamped that move group moves to"),
      BT::InputPort<std::string>("planning_pipeline", "the name of planning pipeline"),
      BT::InputPort<std::string>("planner_id", "the name of planner id in the planning pipeline"),
      BT::InputPort<moveit_msgs::msg::Constraints>("path_constraints", "path constraints"),
      BT::InputPort<std::string>("ik_frame", "the ik frame"),
      BT::OutputPort<moveit_msgs::msg::RobotTrajectory>(
        "trajectory", "The posestamped of the target marker"),
      BT::OutputPort<ComputePathResult::_error_code_id_type>(
        "error_code_id", "Action error code ID"),
    });
  }
};
}  // namespace man2_bt_skill_clients

#endif  // MAN2_BT_SKILL_CLIENTS__COMPUTE_PATH_TO_POSE_ACTION_CLIENT_HPP_
