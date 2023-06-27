#ifndef ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

namespace man2_behavior_tree
{
class ManipulationBehaviorTreeEngine : public nav2_behavior_tree::BehaviorTreeEngine
{
public:
  using BehaviorTreeEngine::BehaviorTreeEngine;

  nav2_behavior_tree::BtStatus
  run_loop(BT::Tree* tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
           std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));
};

}  // namespace man2_behavior_tree

#endif  // ROS2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
