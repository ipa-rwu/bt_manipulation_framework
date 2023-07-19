#include "man2_bt_skills/detect_aruco_marker.hpp"
#include "man2_plugin_base/perception_base.hpp"

namespace perception_skills
{
DetectArucoMarker::DetectArucoMarker(const std::string& xml_tag_name,
                                     const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(xml_tag_name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "Initialising Execute Node.");
  sub_markers_ = node_->create_subscription<aruco_msgs::msg::MarkerArray>(
      "marker_publisher/markers", 10,
      std::bind(&DetectArucoMarker::getMarkerCallback, this, std::placeholders::_1));
  online_.store(false);

  marker_publisher_ = node_->create_publisher<visualization_msgs::msg::Marker>("~/marker", 10);
}

void DetectArucoMarker::getMarkerCallback(const aruco_msgs::msg::MarkerArray& msg)
{
  if (online_.load())
  {
    int marker_id = marker_id_.load();
    RCLCPP_INFO(node_->get_logger(), "Checking for marker ID: %d", marker_id);

    auto func = [marker_id](const aruco_msgs::msg::Marker& x) { return (int)x.id == marker_id; };
    // RCLCPP_INFO(node_->get_logger(), "MarkerArray: %s", aruco_msgs::msg::to_yaml(msg).c_str());
    // Find marker with our ID
    auto res = std::find_if(msg.markers.begin(), msg.markers.end(), std::move(func));

    // If marker with our ID is found, store it for later processing.
    if (res != msg.markers.end())
    {
      geometry_msgs::msg::PoseStamped marker_posestamped;
      marker_posestamped.header = res->header;
      marker_posestamped.pose = res->pose.pose;
      res_markers_.push_back(marker_posestamped);
      counter_++;
      RCLCPP_INFO(node_->get_logger(), "Counter: %d", counter_.load());
    }
    if (required_poses_.load() == counter_.load())
    {
      online_.store(false);
      timer_->cancel();
    }
  }
}

BT::PortsList DetectArucoMarker::providedPorts()
{
  return {
    BT::InputPort<int>("marker_id"),
    BT::InputPort<int>("timeout", 1000, "Timeout in milliseconds."),
  };
}

BT::NodeStatus DetectArucoMarker::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "Initialising Aruco Detection.");
  if (!getInput<int>("marker_id").has_value())
  {
    RCLCPP_ERROR(node_->get_logger(), "No marker id provided");
    return BT::NodeStatus::FAILURE;
  }
  res_markers_.clear();
  marker_id_.store(getInput<int>("marker_id").value());
  int timeout = getInput<int>("timeout").value();

  online_.store(true);
  counter_.store(0);
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(timeout), [this]() {
    if (online_.load())
    {
      RCLCPP_INFO(node_->get_logger(), "Aruco Detection timed out.");
      online_.store(false);
      error_.store(true);
    }
    timer_->cancel();
  });
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectArucoMarker::onRunning()
{
  RCLCPP_INFO(node_->get_logger(), "on running");
}

void DetectArucoMarker::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Halting aruco marker detection.");
  online_.store(false);
  error_.store(true);
  timer_->cancel();
}

}  // namespace perception_skills

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(perception_skills::DetectArucoMarker(const std::string& xml_tag_name,
//                                                             const BT::NodeConfiguration& config),
//                        man2_plugin_base::PerceptionBase)

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(BT::CreateManifest<perception_skills::DetectArucoMarker>(
                              "DetectArucoMarker",
                              perception_skills::DetectArucoMarker::providedPorts()),
                          BT::CreateBuilder<perception_skills::DetectArucoMarker>());
}
