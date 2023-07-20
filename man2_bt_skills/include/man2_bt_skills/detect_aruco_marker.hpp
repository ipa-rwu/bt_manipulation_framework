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

#ifndef MAN2_BT_SKILLS__DETECT_ARUCO_MARKER_HPP_
#define MAN2_BT_SKILLS__DETECT_ARUCO_MARKER_HPP_

#include <behaviortree_cpp/action_node.h>

#include <aruco_msgs/msg/marker_array.hpp>
#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace perception_skills
{
class DetectArucoMarker : public BT::StatefulActionNode
{
public:
  DetectArucoMarker(const std::string & xml_tag_name, const BT::NodeConfiguration & config);

  /**
   * @brief
   *
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief
   *
   * @return BT::NodeStatus
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief
   *
   */
  void onHalted() override;

  static BT::PortsList providedPorts();
  // {
  //   return BT::PortsList({});
  // }

protected:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr sub_markers_;
  std::atomic_bool online_;
  std::atomic_int marker_id_;
  std::atomic_int counter_;
  std::atomic_int required_poses_;
  std::atomic_bool error_;
  std::vector<geometry_msgs::msg::PoseStamped> res_markers_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string planning_group_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

private:
  void getMarkerCallback(const aruco_msgs::msg::MarkerArray & msg);
};

}  // namespace perception_skills

#endif  // MAN2_BT_SKILLS__DETECT_ARUCO_MARKER_HPP_
