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

#ifndef DETECT_ARUCO_MARKER_SLILL__DETECT_ARUCO_MARKER_ACTION_SERVER_HPP_
#define DETECT_ARUCO_MARKER_SLILL__DETECT_ARUCO_MARKER_ACTION_SERVER_HPP_

#include <aruco_msgs/msg/marker_array.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "detect_aruco_marker_skill/action/aruco_marker_detection.hpp"
#include "man2_skill_server_core/skill_action_server_lifecycle_core.hpp"

namespace perception_skills
{
struct RosParameters
{
  std::string default_sub_marker_topic;
  int default_timeout;
  int default_required_pose_num;
  int sub_rate;

  std::string ns = "";

  void declareRosParameters(const nav2_util::LifecycleNode::SharedPtr & node)
  {
    if (!node->has_parameter(ns + "timeout")) {
      node->declare_parameter(ns + "timeout", 5000);
    }

    if (!node->has_parameter(ns + "required_pose_num")) {
      node->declare_parameter(ns + "required_pose_num", 3);
    }

    if (!node->has_parameter(ns + "sub_marker_topic")) {
      node->declare_parameter(ns + "sub_marker_topic", std::string("/markers"));
    }

    if (!node->has_parameter(ns + "sub_rate")) {
      node->declare_parameter(ns + "sub_rate", 10);
    }
  }

  void loadRosParameters(const nav2_util::LifecycleNode::SharedPtr & node)
  {
    node->get_parameter(ns + "timeout", default_timeout);
    node->get_parameter(ns + "required_pose_num", default_required_pose_num);
    node->get_parameter(ns + "sub_marker_topic", default_sub_marker_topic);
    node->get_parameter(ns + "sub_rate", sub_rate);

    RCLCPP_INFO(
      rclcpp::get_logger(node->get_name()),
      "loadRosParameters: default timeout: %ld, required_pose_num: %ld, sub_marker_topic: %s",
      node->get_parameter("timeout").as_int(), node->get_parameter("required_pose_num").as_int(),
      node->get_parameter("sub_marker_topic").value_to_string().c_str());
  }
};

class DetectArucoMarkerActionServer : public ros2_skill_server_core::SkillActionServerLifecycleCore<
                                        detect_aruco_marker_skill::action::ArucoMarkerDetection>
{
public:
  using ActionT = detect_aruco_marker_skill::action::ArucoMarkerDetection;

  DetectArucoMarkerActionServer(
    const std::string & action_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~DetectArucoMarkerActionServer();

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  void execution() override;

protected:
  void getMarkerCallback(const aruco_msgs::msg::MarkerArray & /*msg*/){};

  void initial();

private:
  void processMsg(const aruco_msgs::msg::MarkerArray & msg, int marker_id, int required_pose_num);

  geometry_msgs::msg::PoseStamped getAveragePosition(
    std::vector<geometry_msgs::msg::PoseStamped> poses);

  geometry_msgs::msg::PoseStamped getResult(
    const std::vector<geometry_msgs::msg::PoseStamped> & poses);

  void timerCallback();

  rclcpp::Subscription<aruco_msgs::msg::MarkerArray>::SharedPtr sub_markers_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::WaitSet::SharedPtr wait_set_;

  std::shared_ptr<RosParameters> parameters_;

  std::string action_name_;

  std::atomic_int marker_id_;
  std::atomic_int timeout_;
  std::atomic_int sub_rate_;
  std::atomic_int required_pose_num_;
  std::atomic_int counter_;
  std::vector<geometry_msgs::msg::PoseStamped> res_markers_;

  std::atomic_bool online_;
  std::atomic_bool timeout_flag_;
};

}  // namespace perception_skills

#endif  // DETECT_ARUCO_MARKER_SLILL__DETECT_ARUCO_MARKER_ACTION_SERVER_HPP_
