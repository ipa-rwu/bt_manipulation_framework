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

#ifndef MAN2_BT_OPERATOR__BT_OPERATOR_HPP_
#define MAN2_BT_OPERATOR__BT_OPERATOR_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "man2_behavior_tree/behavior_tree_engine.hpp"
#include "man2_msgs/action/run_application.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"

#ifdef ZMQ_FOUND
#endif

namespace man2_bt_operator
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("bt_operator");

class BTOperator : public nav2_util::LifecycleNode
{
  /**
   * @brief Declare and load ros parameters
   *
   */
  struct RosParameters
  {
    std::vector<std::string> default_plugin_lib_names;
    std::map<std::string, std::vector<std::string>> customized_plugin_lib_names;
    std::string default_bt_xml_filename;
    std::string current_bt_xml_filename;
    bool print_bt_status;
    bool connect_to_groot2;

    std::string ns = "";

    void declareRosParameters(const nav2_util::LifecycleNode::SharedPtr & node)
    {
      node->declare_parameter(ns + "current_bt_xml_filename", rclcpp::PARAMETER_STRING);
      node->declare_parameter(ns + "default_bt_xml_filename", rclcpp::PARAMETER_STRING);
      node->declare_parameter(ns + "default_plugin_lib_names", rclcpp::PARAMETER_STRING_ARRAY);
      node->declare_parameter(ns + "print_bt_status", rclcpp::PARAMETER_BOOL);
      node->declare_parameter(ns + "connect_to_groot2", rclcpp::PARAMETER_BOOL);
      std::string prefix_plugin = "customized_plugin_lib_names.";
      auto all_params = node->get_node_parameters_interface()->get_parameter_overrides();
      for (const auto & param : all_params) {
        std::size_t i = param.first.find(prefix_plugin);
        if (i != std::string::npos) {
          std::string tmp = param.first;
          tmp.erase(i, prefix_plugin.length());
          node->declare_parameter(param.first.c_str(), param.second);
          customized_plugin_lib_names[tmp] = std::vector<std::string>{};
        }
      }
    }

    void loadRosParameters(const nav2_util::LifecycleNode::SharedPtr & node)
    {
      node->get_parameter_or(
        ns + "default_plugin_lib_names", default_plugin_lib_names, std::vector<std::string>{});
      node->get_parameter_or(
        ns + "default_bt_xml_filename", default_bt_xml_filename,
        ament_index_cpp::get_package_share_directory("man2_bt_operator") +
          std::string("/tree/default_bt_xml_filename.xml"));
      node->get_parameter_or(ns + "print_bt_status", print_bt_status, false);
      node->get_parameter_or(ns + "connect_to_groot2", connect_to_groot2, false);

      std::string prefix_plugin = "customized_plugin_lib_names.";
      auto all_params = node->get_node_parameters_interface()->get_parameter_overrides();
      for (const auto & param : all_params) {
        std::size_t i = param.first.find(prefix_plugin);
        if (i != std::string::npos) {
          std::string tmp = param.first;
          tmp.erase(i, prefix_plugin.length());
          if (param.second.get_type() == rclcpp::PARAMETER_STRING_ARRAY) {
            customized_plugin_lib_names[tmp] = node->get_parameter(param.first).as_string_array();
          } else {
            customized_plugin_lib_names[tmp].push_back(
              node->get_parameter(param.first).as_string());
          }
        }
      }

      RCLCPP_INFO(
        LOGGER, "loadRosParameters: default plugins: %s, default_bt_xml_filename: %s",
        node->get_parameter("default_plugin_lib_names").value_to_string().c_str(),
        default_bt_xml_filename.c_str());
    }

    void setParameter(
      const nav2_util::LifecycleNode::SharedPtr & node, const std::string & parameter_name)
    {
      node->set_parameter(rclcpp::Parameter(parameter_name));
    }
  };

public:
  /**
   * @brief Construct a new BTOperator object
   *
   */
  BTOperator();

  /**
   * @brief Destroy the BTOperator object
   *
   */
  ~BTOperator();

protected:
  /**
   * @brief do something during lifecycle node configuration
   *
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override;

  /**
   * @brief do something during lifecycle node activation
   *
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override;

  /**
   * @brief do something during lifecycle node deactivation
   *
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override;

  /**
   * @brief do something during lifecycle node cleaning up
   *
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override;

  /**
   * @brief do something during lifecycle node shutdown
   *
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override;

  /**
   * @brief load a behavior tree xml
   *
   * @param bt_xml_filename
   * @return true
   * @return false
   */
  bool loadBehaviorTree(const std::string & bt_xml_filename);

  void startApplication();

  bool validateGoal(const std::shared_ptr<const man2_msgs::action::RunApplication_Goal> goal);

  using Action = man2_msgs::action::RunApplication;

  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the NavigateToPose action
  std::unique_ptr<ActionServer> action_server_;

  // The wrapper class for the BT functionality
  std::unique_ptr<ros2_behavior_tree::ROS2BehaviorTreeEngine> bt_;

  BT::Tree tree_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  std::shared_ptr<RosParameters> parameters_;

  rclcpp::Time start_time_;

  rclcpp::Node::SharedPtr client_node_;
};

}  // namespace man2_bt_operator
#endif  // MAN_BEHAVIOR_TREE__BT_OPERATOR_HPP_
