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

#include "man2_bt_operator/bt_operator.hpp"

#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

namespace man2_bt_operator
{
BTOperator::BTOperator()
: nav2_util::LifecycleNode("bt_operator", "", rclcpp::NodeOptions()), start_time_(now())
{
  parameters_ = std::make_shared<RosParameters>();

  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r", std::string("__node:=") + std::string(this->get_name()) + "_", "-p",
     "use_sim_time:=" +
       std::string(this->get_parameter("use_sim_time").as_bool() ? "true" : "false"),
     "--"});
  client_node_ = client_node_ = std::make_shared<rclcpp::Node>("_", options);
}

BTOperator::~BTOperator() {}

nav2_util::CallbackReturn BTOperator::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(LOGGER, "Configuring");
  auto node = shared_from_this();
  parameters_->declareRosParameters(node);

  parameters_->loadRosParameters(node);

  bt_loop_duration_ = std::chrono::milliseconds(parameters_->bt_loop_duration);
  default_server_timeout_ = std::chrono::milliseconds(parameters_->server_timeout);

  action_server_ = std::make_unique<ActionServer>(
    get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
    get_node_waitables_interface(), "start_application",
    std::bind(&BTOperator::startApplication, this));

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<ros2_behavior_tree::ROS2BehaviorTreeEngine>();

  // Load default plugins
  bt_->loadDefaultPlugins(parameters_->default_plugin_lib_names);

  bt_->loadAbsolutePlugins(parameters_->customized_plugin_lib_names);

  // add items to blackboard
  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  blackboard_->set<int>("number_recoveries", 0);
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);
  blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", bt_loop_duration_);
  blackboard_->set<std::chrono::milliseconds>("server_timeout", default_server_timeout_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BTOperator::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(LOGGER, "Activating");

  action_server_->activate();

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BTOperator::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating");

  action_server_->deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BTOperator::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(LOGGER, "Cleaning up");

  parameters_.reset();
  blackboard_.reset();
  action_server_.reset();
  tree_.haltTree();
  bt_.reset();
  client_node_.reset();

  RCLCPP_INFO(LOGGER, "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BTOperator::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(LOGGER, "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool BTOperator::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Use previous BT if it is the existing one
  if (parameters_->current_bt_xml_filename == bt_xml_filename) {
    RCLCPP_INFO(LOGGER, "Current BT is as same as request BT");
    return true;
  }

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good()) {
    RCLCPP_ERROR(LOGGER, "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return false;
  }

  auto xml_string =
    std::string(std::istreambuf_iterator<char>(xml_file), std::istreambuf_iterator<char>());

  RCLCPP_INFO(LOGGER, "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_INFO(LOGGER, "Behavior Tree XML: %s", xml_string.c_str());

  tree_ = bt_->createTreeFromText(xml_string, blackboard_);

  parameters_->current_bt_xml_filename = bt_xml_filename;
  parameters_->setParameter(shared_from_this(), "current_bt_xml_filename");

  return true;
}

void BTOperator::startApplication()
{
  auto is_canceling = [this]() {
    if (action_server_ == nullptr) {
      RCLCPP_INFO(LOGGER, "Action server unavailable. Canceling.");
      return true;
    }

    if (!action_server_->is_server_active()) {
      RCLCPP_INFO(LOGGER, "Action server is inactive. Canceling.");
      return true;
    }
    if (!validateGoal(action_server_->get_current_goal())) {
      RCLCPP_INFO(LOGGER, "Goal is invalid");
      return true;
    }

    return action_server_->is_cancel_requested();
  };

  auto bt_xml_filename = action_server_->get_current_goal()->behavior_tree_filename;

  // Empty id in request is default for backward compatibility
  if (bt_xml_filename.empty() || bt_xml_filename == "" || bt_xml_filename == "None") {
    bt_xml_filename = parameters_->default_bt_xml_filename;
  }

  if (!loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      LOGGER, "BT file not found: %s. Current: %s, Application canceled", bt_xml_filename.c_str(),
      parameters_->current_bt_xml_filename.c_str());
    action_server_->terminate_current();
    return;
  }

  std::shared_ptr<Action::Feedback> feedback_msg = std::make_shared<Action::Feedback>();

  int application_loop_counter = 0;
  auto on_loop = [&]() {
    if (action_server_->is_preempt_requested()) {
      RCLCPP_INFO(LOGGER, "Received goal preemption request");
      application_loop_counter = 0;
      action_server_->accept_pending_goal();
    }
    application_loop_counter++;
    // action server feedback

    int recovery_count = 0;
    blackboard_->get<int>("number_recoveries", recovery_count);
    feedback_msg->number_of_recoveries = recovery_count;
    feedback_msg->execution_time = now() - start_time_;
    RCLCPP_INFO(LOGGER, "Application loop times: %d", application_loop_counter);
    action_server_->publish_feedback(feedback_msg);
  };

  // Execute the BT that was previously created in the configure step

  start_time_ = now();
  ros2_behavior_tree::BtStatus rc;
  if (action_server_->get_current_goal()->run_in_loop) {
    rc = bt_->run_loop(
      &tree_, on_loop, is_canceling,
      std::chrono::milliseconds(action_server_->get_current_goal()->sleep));
  } else {
    rc = bt_->run(&tree_, on_loop, is_canceling);
  }

  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not
  // needed.
  tree_.haltTree();

  switch (rc) {
    case ros2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(LOGGER, "Application succeeded");
      action_server_->succeeded_current();
      break;

    case ros2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(LOGGER, "Application failed");
      action_server_->terminate_current();
      break;

    case ros2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(LOGGER, "Application canceled");
      action_server_->terminate_all();
      break;
  }
}

bool BTOperator::validateGoal(
  const std::shared_ptr<const man2_msgs::action::RunApplication_Goal> goal)
{
  if (goal->run_in_loop) {
    if (goal->sleep == 0) {
      RCLCPP_ERROR(LOGGER, "Please set sleep time in goal if the application runs repeatedly");
      return false;
    }
  }
  return true;
}

}  // namespace man2_bt_operator
