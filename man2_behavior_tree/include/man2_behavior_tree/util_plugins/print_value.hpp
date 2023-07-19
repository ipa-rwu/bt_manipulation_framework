#ifndef ROS2_BEHAVIOR_TREE__util_plugins_PRINT_VALUE_HPP_
#define ROS2_BEHAVIOR_TREE__util_plugins_PRINT_VALUE_HPP_

#include "behaviortree_cpp/bt_factory.h"

namespace util_plugins
{
class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();
};
}  // namespace util_plugins

#endif  // ROS2_BEHAVIOR_TREE__util_plugins_PRINT_VALUE_HPP_
