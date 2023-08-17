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

#ifndef MAN2_BT_SKILL_CLIENTS__SET_PATH_CONSTRAINTS_SERVICE_CLIENT_HPP_
#define MAN2_BT_SKILL_CLIENTS__SET_PATH_CONSTRAINTS_SERVICE_CLIENT_HPP_

#include "man2_behavior_tree/bt_conversions.hpp"
#include "man2_behavior_tree/bt_service_client_node.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit_skills/action/execute_trajectory.hpp"
#include "moveit_skills/srv/set_path_constrains.hpp"

namespace man2_bt_skill_clients
{
class SetPathConstrainsServiceClient
: public ros2_behavior_tree::BtServiceClientNode<moveit_skills::srv::SetPathConstrains>
{
public:
  using Service = moveit_skills::srv::SetPathConstrains;

  SetPathConstrainsServiceClient(
    const std::string & service_node_name, const BT::NodeConfiguration & conf)
  : ros2_behavior_tree::BtServiceClientNode<Service>(service_node_name, conf)
  {
  }

  void on_tick() override;

  BT::NodeStatus on_completion(std::shared_ptr<Service::Response> response) override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("file_path", "The path of a path constrains yaml file"),
      BT::OutputPort<moveit_msgs::msg::Constraints>("path_constraints", "path constraints"),
      BT::OutputPort<Service::Response::_error_code_id_type>(
        "error_code_id", "Serevr error code ID"),
    });
  }
};
}  // namespace man2_bt_skill_clients

#endif  // MAN2_BT_SKILL_CLIENTS__SET_PATH_CONSTRAINTS_SERVICE_CLIENT_HPP_
