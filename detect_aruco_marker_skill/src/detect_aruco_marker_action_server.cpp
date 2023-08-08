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

#include "detect_aruco_marker_skill/detect_aruco_marker_action_server.hpp"

namespace perception_skills
{
DetectArucoMarkerActionServer::DetectArucoMarkerActionServer(
  const std::string & action_name, const rclcpp::NodeOptions & options)
: ros2_skill_server_core::SkillActionServerLifecycleCore<
    detect_aruco_marker_skill::action::ArucoMarkerDetection>(action_name, options),
  action_name_(action_name)
{
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(this->get_name()));
  parameters_ = std::make_shared<RosParameters>();
  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
}

DetectArucoMarkerActionServer::~DetectArucoMarkerActionServer() {}

nav2_util::CallbackReturn DetectArucoMarkerActionServer::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto node = shared_from_this();

  parameters_->declareRosParameters(node);
  parameters_->loadRosParameters(node);

  RCLCPP_INFO(
    *logger_, "loadRosParameters: default timeout: %d, required_pose_num: %d, sub_marker_topic: %s",
    parameters_->default_timeout, parameters_->default_required_pose_num,
    parameters_->default_sub_marker_topic.c_str());
  parameters_->loadRosParameters(node);

  // Create the action server that we implement with our followPath method
  action_server_ = std::make_unique<ActionServer>(
    node, action_name_, std::bind(&DetectArucoMarkerActionServer::execution, this), nullptr,
    std::chrono::milliseconds(500), true);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn DetectArucoMarkerActionServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(*logger_, "Cleaning up...");
  action_server_.reset();
  initial();
  return nav2_util::CallbackReturn::SUCCESS;
}

void DetectArucoMarkerActionServer::initial()
{
  counter_.store(0);
  timeout_flag_.store(false);
  res_markers_.clear();
  online_.store(true);
  counter_.store(0);
  wait_set_.reset();
  timer_.reset();
}

void DetectArucoMarkerActionServer::execution()
{
  auto node = shared_from_this();
  parameters_->loadRosParameters(node);

  auto goal = getCurrentGoal();
  auto result = std::make_shared<ActionT::Result>();

  initial();

  if (goal->marker_id) {
    if (goal->sub_topic_name != "") {
      parameters_->default_sub_marker_topic = goal->sub_topic_name;
    }
    if (goal->timeout != 0) {
      parameters_->default_timeout = goal->timeout;
    }
    if (goal->required_pose_num != 0) {
      parameters_->default_required_pose_num = goal->required_pose_num;
    }

    sub_markers_ = node->create_subscription<aruco_msgs::msg::MarkerArray>(
      parameters_->default_sub_marker_topic, 10,
      std::bind(&DetectArucoMarkerActionServer::getMarkerCallback, this, std::placeholders::_1));

    timer_ = node->create_wall_timer(
      std::chrono::milliseconds(parameters_->default_timeout),
      std::bind(&DetectArucoMarkerActionServer::timerCallback, this), callback_group_);

    wait_set_ = std::make_shared<rclcpp::WaitSet>();
    wait_set_->add_subscription(sub_markers_);
    wait_set_->add_timer(timer_);
  } else {
    result->error_code_id = result->INVALID_INPUT;
    sendFaildResult(result);
    return;
  }

  while (rclcpp::ok() && !action_server_->is_cancel_requested() &&
         !action_server_->is_preempt_requested() && online_.load() == true) {
    const auto wait_result =
      wait_set_->wait(std::chrono::milliseconds(1000 / parameters_->sub_rate));
    switch (wait_result.kind()) {
      case rclcpp::WaitResultKind::Ready: {
        if (wait_result.get_wait_set().get_rcl_wait_set().timers[0U]) {
          timer_->execute_callback();
        } else {
          aruco_msgs::msg::MarkerArray msg;
          rclcpp::MessageInfo msg_info;
          if (sub_markers_->take(msg, msg_info)) {
            // msg from subscription will be processed here
            processMsg(msg, goal->marker_id, parameters_->default_required_pose_num);
          }
        }
        break;
      }
      case rclcpp::WaitResultKind::Timeout:
        if (rclcpp::ok()) {
          RCLCPP_WARN(*logger_, "No message received after given wait-time");
        }
        break;
      default:
        RCLCPP_ERROR(*logger_, "Error. Wait-set failed.");
        result->error_code_id = result->WAITSER_ERROR;
        sendFaildResult(result);
        return;
    }
  }

  if (!online_.load()) {
    RCLCPP_INFO(*logger_, "Aruco Detection done.");
    if (timeout_flag_.load()) {
      if (counter_ < parameters_->default_required_pose_num) {
        RCLCPP_ERROR(*logger_, "Timed out and insufficient marker poses returned.");
        result->error_code_id = result->TIMEOUT;
        sendFaildResult(result);
        return;
      }
    }
  }

  // Process positions
  auto marker_pose = getResult(res_markers_);
  RCLCPP_DEBUG(*logger_, "marker_pose: %s", geometry_msgs::msg::to_yaml(marker_pose).c_str());

  result->pose = marker_pose;
  sendSucceededResult(result);
}

void DetectArucoMarkerActionServer::timerCallback()
{
  RCLCPP_INFO(*logger_, "Just timeout");
  timeout_flag_.store(true);
  wait_set_->remove_subscription(sub_markers_);
  wait_set_->remove_timer(timer_);
}

void DetectArucoMarkerActionServer::processMsg(
  const aruco_msgs::msg::MarkerArray & msg, int marker_id, int required_pose_num)
{
  RCLCPP_INFO(*logger_, "Checking for marker ID: %d", marker_id);

  auto func = [marker_id](const aruco_msgs::msg::Marker & x) { return (int)x.id == marker_id; };
  auto res = std::find_if(msg.markers.begin(), msg.markers.end(), std::move(func));

  // If marker with our ID is found, store it for later processing.
  if (res != msg.markers.end()) {
    geometry_msgs::msg::PoseStamped marker_posestamped;
    marker_posestamped.header = res->header;
    marker_posestamped.pose = res->pose.pose;
    res_markers_.push_back(marker_posestamped);
    counter_++;
    RCLCPP_INFO(*logger_, "Counter: %d", counter_.load());
  }
  if (required_pose_num == counter_.load()) {
    online_.store(false);
    timer_->cancel();
    wait_set_->remove_subscription(sub_markers_);
  }
}

geometry_msgs::msg::PoseStamped DetectArucoMarkerActionServer::getAveragePosition(
  std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  geometry_msgs::msg::PoseStamped average_pose = std::accumulate(
    poses.begin(), poses.end(), geometry_msgs::msg::PoseStamped(),
    [](geometry_msgs::msg::PoseStamped & a, geometry_msgs::msg::PoseStamped & b) {
      a.pose.position.x += b.pose.position.x;
      a.pose.position.y += b.pose.position.y;
      a.pose.position.z += b.pose.position.z;

      a.pose.orientation.x += b.pose.orientation.x;
      a.pose.orientation.y += b.pose.orientation.y;
      a.pose.orientation.z += b.pose.orientation.z;
      a.pose.orientation.w += b.pose.orientation.w;
      return a;
    });

  average_pose.header = poses[0U].header;
  average_pose.pose.position.x /= poses.size();
  average_pose.pose.position.y /= poses.size();
  average_pose.pose.position.z /= poses.size();

  average_pose.pose.orientation.x /= poses.size();
  average_pose.pose.orientation.y /= poses.size();
  average_pose.pose.orientation.z /= poses.size();
  average_pose.pose.orientation.w /= poses.size();

  return average_pose;
}

geometry_msgs::msg::PoseStamped DetectArucoMarkerActionServer::getResult(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  // create average position
  return getAveragePosition(poses);
}

}  // namespace perception_skills
