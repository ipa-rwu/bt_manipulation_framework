#ifndef ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"

using namespace std::chrono_literals;

namespace ros2_behavior_tree
{
enum class BtStatus
{
  SUCCEEDED,
  FAILED,
  CANCELED
};
class ROS2BehaviorTreeEngine
{
public:
  explicit ROS2BehaviorTreeEngine(const std::vector<std::string>& plugin_libraries);
  virtual ~ROS2BehaviorTreeEngine()
  {
  }

  ros2_behavior_tree::BtStatus
  run_loop(BT::Tree* tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
           std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  /**
   * @brief Function to create a BT from a XML string
   * @param xml_string XML string representing BT
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromText(const std::string& xml_string, BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to create a BT from an XML file
   * @param file_path Path to BT XML file
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromFile(const std::string& file_path, BT::Blackboard::Ptr blackboard);

protected:
  // The factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
