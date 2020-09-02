#include "man_behavior_tree_nodes/bt_engine.hpp"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include <memory>
#include <string>

namespace man_behavior_tree_nodes
{
BT_Engine::BT_Engine(const std::vector<std::string> & plugin_libraries)
{
  BT::SharedLibrary loader;
  for (const auto & p : plugin_libraries) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }
}

BT::Tree BT_Engine::buildTreeFromText(const std::string & xml_string,
                                      BT::Blackboard::Ptr blackboard)
{
  BT::XMLParser p(factory_);
  p.loadFromText(xml_string);
  return p.instantiateTree(blackboard);
}



BtStatus BT_Engine::run(
  BT::Tree * tree
  // ,std::function<void()> onLoop,
  // std::function<bool()> cancelRequested
  // ,std::chrono::milliseconds loopTimeout
  )
{
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  ros::Rate r(1);
  // Loop until something happens with ROS or the node completes
  while (ros::ok()) 
  // while (ros::ok()&& result == BT::NodeStatus::RUNNING) 
  {
    // if (cancelRequested()) {
    //   tree->rootNode()->halt();
    //   return BtStatus::CANCELED;
    // }

    result = tree->rootNode()->executeTick();
    r.sleep();
    // onLoop();

    // loopRate.sleep();
  }

  // return (result == BT::NodeStatus::SUCCESS) ? BtStatus::SUCCEEDED : BtStatus::FAILED;
}


} // namespace man_behavior_tree


