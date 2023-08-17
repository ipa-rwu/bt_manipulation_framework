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

#ifndef MAN2_BT_SKILL_CLIENTS__DETECT_ARUCO_MARKER_ACTION_CLIENT_HPP_
#define MAN2_BT_SKILL_CLIENTS__DETECT_ARUCO_MARKER_ACTION_CLIENT_HPP_

#include "detect_aruco_marker_skill/action/aruco_marker_detection.hpp"
#include "man2_behavior_tree/bt_action_client_node.hpp"
#include "man2_behavior_tree/bt_conversions.hpp"

namespace man2_bt_skill_clients
{
class DetectArucoMarkerActionClient : public ros2_behavior_tree::BtActionClientNode<
                                        detect_aruco_marker_skill::action::ArucoMarkerDetection>
{
public:
  using Action = detect_aruco_marker_skill::action::ArucoMarkerDetection;
  using ActionResult = detect_aruco_marker_skill::action::ArucoMarkerDetection::Result;

  DetectArucoMarkerActionClient(
    const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : ros2_behavior_tree::BtActionClientNode<Action>(xml_tag_name, "detect_aruco_marker", conf)
  {
  }

  void on_tick() override;

  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<int>("marker_id", "target Marker ID"),
      BT::InputPort<int>(
        "timeout", "Time of waiting for result; default value defined as ros parameter"),
      BT::InputPort<std::string>(
        "sub_topic_name",
        "The topic that publishing all detected marker positions; default value defined as ros "
        "parameter"),
      BT::InputPort<int>(
        "required_pose_num",
        "The number of maker positions for calculating average; default value defined as ros "
        "parameter"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>(
        "marker_pose", "The posestamped of the target marker"),
      BT::OutputPort<ActionResult::_error_code_id_type>("error_code_id", "Action error code ID"),
    });
  }
};
}  // namespace man2_bt_skill_clients

#endif  //MAN2_BT_SKILL_CLIENTS__DETECT_ARUCO_MARKER_ACTION_CLIENT_HPP_
