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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>

#include <aruco_msgs/msg/marker_array.hpp>
#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
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
   * @brief Configuration during state change from IDLE to RUNNING
   *
   * @return BT::NodeStatus
   */
  BT::NodeStatus onStart() override;

  /**
   * @brief Do something during RUNNING
   *
   * @return BT::NodeStatus
   */
  BT::NodeStatus onRunning() override;

  /**
   * @brief Do something when cancel
   *
   */
  void onHalted() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("marker_id"),
      BT::InputPort<std::string>(
        "sub_topic_name", "Subscribe to a topic that publishing positions of any markers"),
      BT::InputPort<int>("timeout", 1000, "Timeout in milliseconds."),
      BT::InputPort<int>("sub_rate", 10, "Subsctribe rate (HZ)"),
      BT::InputPort<std::string>("pub_topic_name", "Topic name of publishing specific marker ID"),
      BT::InputPort<std::string>("frame", "Target frame"),
      BT::InputPort<int>(
        "required_pose_num", 3, "Number of pose numbers for calculating the average"),
    };
  }

protected:
  geometry_msgs::msg::PoseStamped getResult(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr sub_markers_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  std::atomic_bool online_;
  std::atomic_int counter_;
  std::atomic_bool timeout_flag_;
  std::vector<geometry_msgs::msg::PoseStamped> res_markers_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // blackboard variables
  std::string sub_topic_name_;
  std::string pub_topic_name_;
  std::string frame_;
  std::atomic_int marker_id_;
  std::atomic_int timeout_;
  std::atomic_int sub_rate_;
  std::atomic_int required_pose_num_;

private:
  /**
 * @brief Subscribe callback; When using WaitSet, msg won't be processed here
 *
 * @param msg
 */
  void getMarkerCallback(const aruco_msgs::msg::MarkerArray & msg){};

  /**
   * @brief Timer callback, do something after timeout
   *
   */
  void timerCallback();

  /**
 * @brief Check if set variables in BT
 *
 * @return true
 * @return false
 */
  bool checkPorts();

  /**
   * @brief Process msg from subscription
   *
   * @param msg
   */
  void processMsg(const aruco_msgs::msg::MarkerArray & msg);

  geometry_msgs::msg::PoseStamped frameTransfer(
    const geometry_msgs::msg::PoseStamped & pose, const std::string target_frame);

  geometry_msgs::msg::PoseStamped getAveragePosition(
    std::vector<geometry_msgs::msg::PoseStamped> poses);

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::WaitSet::SharedPtr wait_set_;
};

}  // namespace perception_skills

#endif  // MAN2_BT_SKILLS__DETECT_ARUCO_MARKER_HPP_
