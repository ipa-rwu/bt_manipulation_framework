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

#ifndef MAN2_BT_SKILL_CLIENTS__PRINT_POSE_HPP_
#define MAN2_BT_SKILL_CLIENTS__PRINT_POSE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace man2_bt_skill_clients
{
class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string & name, const BT::NodeConfig & config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("msg_string"),
      BT::InputPort<moveit_msgs::msg::RobotTrajectory>("msg_trajectory"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("msg_posestamped"),
      BT::InputPort<geometry_msgs::msg::Pose>("msg_pose"),
    };
  }
};

}  // namespace man2_bt_skill_clients

#endif  //MAN2_BT_SKILL_CLIENTS__PRINT_POSE_HPP_
