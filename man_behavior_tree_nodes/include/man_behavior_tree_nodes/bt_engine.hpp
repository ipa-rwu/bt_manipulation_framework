#ifndef MAN_BEHAVIOR_TREE__BT_ENGINE_HPP_
#define MAN_BEHAVIOR_TREE__BT_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"

namespace man_behavior_tree
{

class BT_Engine
{
explicit BT_Engine(const std::vector<std::string> & plugin_libraries);
virtual ~BT_Engine() {}

public:
  BT::Tree buildTreeFromText(
    const std::string & xml_string,
    BT::Blackboard::Ptr blackboard);

  // In order to re-run a Behavior Tree, we must be able to reset all nodes to the initial state
  void haltAllActions(BT::TreeNode * root_node)
  {
    // this halt signal should propagate through the entire tree.
    root_node->halt();

    // but, just in case...
    auto visitor = [](BT::TreeNode * node) {
        if (node->status() == BT::NodeStatus::RUNNING) {
          node->halt();
        }
      };
    BT::applyRecursiveVisitor(root_node, visitor);
    }

protected:
  // The factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
  
};

}
#endif  // MAN_BEHAVIOR_TREE__BT_ENGINE_HPP_
