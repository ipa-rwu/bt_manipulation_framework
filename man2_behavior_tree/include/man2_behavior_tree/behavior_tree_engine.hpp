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
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"

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
  explicit ROS2BehaviorTreeEngine();
  virtual ~ROS2BehaviorTreeEngine()
  {
  }

  /**
   * @brief Load BT plugin from current package
   *
   * @param plugin_libraries
   */
  void loadDefaultPlugins(const std::vector<std::string>& plugin_libraries);

  /**
   * @brief Load BT plugins from absolute path
   *
   * @param plugin_libraries
   */
  void loadAbsolutePlugins(const std::vector<std::string>& plugin_libraries);

  void loadAbsolutePlugins(const std::map<std::string, std::vector<std::string>> package_plugin_map);

  /**
   * @brief Run an entire BT in loops
   *
   * @param tree
   * @param onLoop
   * @param cancelRequested
   * @param loopTimeout
   * @return ros2_behavior_tree::BtStatus
   */
  ros2_behavior_tree::BtStatus
  run_loop(BT::Tree* tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
           std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  /**
   * @brief Run an entire BT one time
   *
   * @param tree
   * @param onLoop
   * @param cancelRequested
   * @param loopTimeout
   * @return ros2_behavior_tree::BtStatus
   */
  ros2_behavior_tree::BtStatus
  run(BT::Tree* tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
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

  /**
   * @brief Save BT log into file (btlog)
   *
   * @param tree
   * @param path for saving log
   */
  void saveLogIntoBtlog(const BT::Tree& tree, const std::string& path);

protected:
  // The factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
};

}  // namespace ros2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
