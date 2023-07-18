#include "man2_behavior_tree/util_plugins/print_value.hpp"

namespace util_plugins
{
PrintValue::PrintValue(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
}

BT::NodeStatus PrintValue::tick()
{
  std::string msg;
  if (getInput("message", msg))
  {
    std::cout << "PrintValue: " << msg << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    std::cout << "PrintValue FAILED " << std::endl;
    return BT::NodeStatus::FAILURE;
  }
}

BT::PortsList PrintValue::providedPorts()
{
  return { BT::InputPort<std::string>("message") };
}

}  // namespace util_plugins

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerBuilder(BT::CreateManifest<util_plugins::PrintValue>(
                              "PrintValue", util_plugins::PrintValue::providedPorts()),
                          BT::CreateBuilder<util_plugins::PrintValue>());
}
