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

#ifndef MAN2_BT_SKILL_CLIENTS__EXECUTE_TRAJECTORY_ACTION_CLIENT_HPP_
#define MAN2_BT_SKILL_CLIENTS__EXECUTE_TRAJECTORY_ACTION_CLIENT_HPP_

#include "man2_behavior_tree/bt_action_client_node.hpp"
#include "man2_behavior_tree/bt_conversions.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_skills/action/execute_trajectory.hpp"

namespace man2_bt_skill_clients
{
class ExecuteTrajectoryActionClient
: public ros2_behavior_tree::BtActionClientNode<moveit_skills::action::ExecuteTrajectory>
{
public:
  using Action = moveit_skills::action::ExecuteTrajectory;
  using ActionResult = moveit_skills::action::ExecuteTrajectory::Result;

  ExecuteTrajectoryActionClient(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : ros2_behavior_tree::BtActionClientNode<Action>(xml_tag_name, "execute_trajectory", conf)
  {
  }

  void on_tick() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("target_group", "The target move group"),
      BT::InputPort<moveit_msgs::msg::RobotTrajectory>(
        "trajectory", "Tryjectory that will be executed"),
      BT::OutputPort<ActionResult::_error_code_id_type>("error_code_id", "Action error code ID"),
    });
  }
};
}  // namespace man2_bt_skill_clients

#endif  // MAN2_BT_SKILL_CLIENTS__EXECUTE_TRAJECTORY_ACTION_CLIENT_HPP_
