// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Florian Gramss
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

#ifndef MAN2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
#define MAN2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "rclcpp/rclcpp.hpp"

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
  ROS2BehaviorTreeEngine();
  virtual ~ROS2BehaviorTreeEngine() {}

  /**
   * @brief Load BT plugin from current package
   *
   * @param plugin_libraries
   */
  void loadDefaultPlugins(const std::vector<std::string> & plugin_libraries);

  /**
   * @brief Load BT plugins from absolute path
   *
   * @param plugin_libraries
   */
  void loadAbsolutePlugins(const std::vector<std::string> & plugin_libraries);

  void loadAbsolutePlugins(
    const std::map<std::string, std::vector<std::string>> package_plugin_map);

  /**
   * @brief Run an entire BT in loops
   *
   * @param tree
   * @param onLoop
   * @param cancelRequested
   * @param loopTimeout
   * @return ros2_behavior_tree::BtStatus
   */
  ros2_behavior_tree::BtStatus run_loop(
    BT::Tree * tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
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
  ros2_behavior_tree::BtStatus run(
    BT::Tree * tree, std::function<void()> onLoop, std::function<bool()> cancelRequested,
    std::chrono::milliseconds loopTimeout = std::chrono::milliseconds(10));

  /**
   * @brief Function to create a BT from a XML string
   * @param xml_string XML string representing BT
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromText(const std::string & xml_string, BT::Blackboard::Ptr blackboard);

  /**
   * @brief Function to create a BT from an XML file
   * @param file_path Path to BT XML file
   * @param blackboard Blackboard for BT
   * @return BT::Tree Created behavior tree
   */
  BT::Tree createTreeFromFile(const std::string & file_path, BT::Blackboard::Ptr blackboard);

  /**
   * @brief Save BT log into file (btlog)
   *
   * @param tree
   * @param path for saving log
   */
  void saveLogIntoBtlog(const BT::Tree & tree, const std::string & path);

protected:
  // The factory that will be used to dynamically construct the behavior tree
  BT::BehaviorTreeFactory factory_;
};

}  // namespace ros2_behavior_tree

#endif  // MAN2_BEHAVIOR_TREE__BEHAVIOR_TREE_ENGINE_HPP_
