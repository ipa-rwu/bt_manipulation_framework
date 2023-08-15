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

#include "man2_bt_perception_skills/detect_aruco_marker.hpp"

namespace perception_skills
{
DetectArucoMarker::DetectArucoMarker(
  const std::string & xml_tag_name, const BT::NodeConfiguration & config)
: BT::StatefulActionNode(xml_tag_name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.callback_group = callback_group_;

  if (!checkPorts()) {
    online_.store(false);
  }

  timeout_flag_.store(false);

  std::string node_name = node_->get_name();

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(node_name + "detect_aruco_maker"));

  RCLCPP_INFO(*logger_, "Initialising Execute Node.");

  // sub_markers_ = node_->create_subscription<aruco_msgs::msg::MarkerArray>(
  //   "marker", 10, std::bind(&DetectArucoMarker::getMarkerCallback, this, std::placeholders::_1),
  //   subscription_options);

  RCLCPP_INFO(*logger_, "subscribe to marker");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());

  online_.store(false);
}

bool DetectArucoMarker::checkPorts()
{
  if (!getInput<std::string>("sub_topic_name").has_value()) {
    RCLCPP_ERROR(*logger_, "Didn't provide marker topic for subscribing");
    return false;
  }
  if (!getInput<std::string>("pub_topic_name").has_value()) {
    RCLCPP_ERROR(*logger_, "Didn't provide topic for publishing required marker ID");
    return false;
  }
  if (!getInput<int>("marker_id").has_value()) {
    RCLCPP_ERROR(*logger_, "No marker id provided");
    return false;
  }

  return true;
}

// void DetectArucoMarker::getMarkerCallback(const aruco_msgs::msg::MarkerArray & msg) {}

void DetectArucoMarker::timerCallback()
{
  RCLCPP_INFO(*logger_, "Just timeout");
  timeout_flag_.store(true);
  online_.store(false);
  wait_set_->remove_subscription(sub_markers_);
  wait_set_->remove_timer(timer_);
}

void DetectArucoMarker::processMsg(const aruco_msgs::msg::MarkerArray & msg)
{
  int marker_id = marker_id_.load();
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
  if (required_pose_num_.load() == counter_.load()) {
    online_.store(false);
    timer_->cancel();
    wait_set_->remove_subscription(sub_markers_);
  }
}

geometry_msgs::msg::PoseStamped DetectArucoMarker::getAveragePosition(
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

geometry_msgs::msg::PoseStamped DetectArucoMarker::frameTransfer(
  const geometry_msgs::msg::PoseStamped & pose, const std::string target_frame)
{
  if (pose.header.frame_id != target_frame) {
    tf2::Quaternion q;
    geometry_msgs::msg::TransformStamped transfer_matrx, t_sum;

    try {
      transfer_matrx =
        tf_buffer_->lookupTransform(target_frame, pose.header.frame_id, tf2::TimePointZero);

      geometry_msgs::msg::PoseStamped final_pose;
      tf2::doTransform(pose, final_pose, transfer_matrx);
      return final_pose;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(*logger_, "TF failed, Exception: %s", ex.what());
    }

  } else {
    return pose;
  }
}

geometry_msgs::msg::PoseStamped DetectArucoMarker::getResult(
  const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  // create average position
  auto average_pose = getAveragePosition(poses);

  // do tf if needed
  auto final_pose = frameTransfer(average_pose, frame_);
  return final_pose;
}

BT::NodeStatus DetectArucoMarker::onStart()
{
  RCLCPP_INFO(*logger_, "Initialising Aruco Detection.");
  if (!checkPorts()) {
    online_.store(false);
    return BT::NodeStatus::FAILURE;
  }

  marker_id_.store(getInput<int>("marker_id").value());
  sub_topic_name_ = getInput<std::string>("sub_topic_name").value();
  pub_topic_name_ = getInput<std::string>("pub_topic_name").value();
  frame_ = getInput<std::string>("frame").value();
  timeout_.store(getInput<int>("timeout").value());
  sub_rate_.store(getInput<int>("sub_rate").value());
  required_pose_num_.store(getInput<int>("required_pose_num").value());

  res_markers_.clear();
  sub_markers_.reset();
  marker_publisher_.reset();
  online_.store(true);
  timeout_flag_.store(false);
  counter_.store(0);

  sub_markers_ = node_->create_subscription<aruco_msgs::msg::MarkerArray>(
    sub_topic_name_, 10,
    std::bind(&DetectArucoMarker::getMarkerCallback, this, std::placeholders::_1));

  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>(pub_topic_name_, 10);

  timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(timeout_), std::bind(&DetectArucoMarker::timerCallback, this),
    callback_group_);

  wait_set_.reset();
  wait_set_ = std::make_shared<rclcpp::WaitSet>();
  wait_set_->add_subscription(sub_markers_);
  wait_set_->add_timer(timer_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectArucoMarker::onRunning()
{
  const auto wait_result = wait_set_->wait(std::chrono::milliseconds(1000 / sub_rate_.load()));
  switch (wait_result.kind()) {
    case rclcpp::WaitResultKind::Ready: {
      if (wait_result.get_wait_set().get_rcl_wait_set().timers[0U]) {
        timer_->execute_callback();
      } else {
        aruco_msgs::msg::MarkerArray msg;
        rclcpp::MessageInfo msg_info;
        if (sub_markers_->take(msg, msg_info)) {
          // msg from subscription will be processed here
          processMsg(msg);
        }
      }
      break;
    }
    case rclcpp::WaitResultKind::Timeout:
      if (rclcpp::ok()) {
        RCLCPP_WARN(*logger_, "No message received after given wait-time");
      }
      break;
    case rclcpp::WaitResultKind::Empty:
      RCLCPP_ERROR(*logger_, "Error. Wait-set failed.");
      break;
  }

  if (!online_.load()) {
    RCLCPP_INFO(*logger_, "Aruco Detection done.");
    if (timeout_.load()) {
      if (counter_ < required_pose_num_) {
        RCLCPP_ERROR(*logger_, "Timed out and insufficient marker poses returned.");
        return BT::NodeStatus::FAILURE;
      }
    }

    // Process positions
    auto marker_pose = getResult(res_markers_);

    RCLCPP_INFO(*logger_, "marker_pose: %s", geometry_msgs::msg::to_yaml(marker_pose).c_str());
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::RUNNING;
  }
}

void DetectArucoMarker::onHalted()
{
  RCLCPP_INFO(*logger_, "Halting aruco marker detection.");
  online_.store(false);
  timer_->cancel();
  wait_set_->remove_subscription(sub_markers_);
}

}  // namespace perception_skills

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(
    BT::CreateManifest<perception_skills::DetectArucoMarker>(
      "DetectArucoMarker", perception_skills::DetectArucoMarker::providedPorts()),
    BT::CreateBuilder<perception_skills::DetectArucoMarker>());
}
