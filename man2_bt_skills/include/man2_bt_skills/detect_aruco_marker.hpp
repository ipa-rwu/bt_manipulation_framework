#ifndef MAN2_BT_SKILLS__PLUGINS__ACTION__DETECT_ARUCO_MARKER_HPP_
#define MAN2_BT_SKILLS__PLUGINS__ACTION__DETECT_ARUCO_MARKER_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>
#include <aruco_msgs/msg/marker_array.hpp>
#include <atomic>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace perception_skills
{
class DetectArucoMarker : public BT::StatefulActionNode
{
public:
  DetectArucoMarker(const std::string& xml_tag_name, const BT::NodeConfiguration& config);

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
  void getMarkerCallback(const aruco_msgs::msg::MarkerArray& msg);
};

}  // namespace perception_skills

#endif
