#include "man2_bt_operator/bt_operator.hpp"

using namespace std::chrono_literals;

namespace man2_bt_operator
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("man2_bt_operator");

BTOperator::BTOperator()
  : nav2_util::LifecycleNode("bt_operator", "", rclcpp::NodeOptions()), start_time_(0)
{
  parameters_ = std::make_shared<RosParameters>();
}

BTOperator::~BTOperator()
{
}

nav2_util::CallbackReturn BTOperator::on_configure(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(LOGGER, "Configuring");
  auto node = shared_from_this();
  parameters_->declareRosParameters(node);

  parameters_->loadRosParameters(node);

  action_server_ =
      std::make_unique<ActionServer>(get_node_base_interface(), get_node_clock_interface(),
                                     get_node_logging_interface(), get_node_waitables_interface(),
                                     "start_application",
                                     std::bind(&BTOperator::startApplication, this));

  // Create the class that registers our custom nodes and executes the BT
  bt_ = std::make_unique<man2_behavior_tree::ManipulationBehaviorTreeEngine>(
      parameters_->plugin_lib_names);

  // add items to blackboard
  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard

  // Put items on the blackboard
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);  // NOLINT
  blackboard_->set<std::chrono::milliseconds>("server_timeout",
                                              std::chrono::milliseconds(10));  // NOLINT
  blackboard_->set<bool>("initial_pose_received", false);                      // NOLINT
  blackboard_->set<int>("number_recoveries", 0);                               // NOLINT

  blackboard_->set<std::map<std::string, float>>("param_float",
                                                 param_float_);  // NOLINT
  blackboard_->set<std::map<std::string, int8_t>>("param_int", param_int_);
  blackboard_->set<std::map<std::string, std::string>>("param_string", param_string_);
  blackboard_->set<std::map<std::string, bool>>("param_bool", param_bool_);
  blackboard_->set<std::string>("world_frame_id", world_frame_id_);
  blackboard_->set<std::string>("group_name_arm", group_name_arm_);
  blackboard_->set<std::string>("group_name_gripper", group_name_gripper_);
  blackboard_->set<std::string>("end_effector", end_effector_name_);
  blackboard_->set<bool>("first_time", first_time_);  // NOLINT

  // load bt xml file

  if (!loadBehaviorTree(parameters_->default_bt_xml_filename))
  {
    RCLCPP_ERROR(LOGGER, "Error loading XML file: %s", parameters_->default_bt_xml_filename.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

#ifdef ZMQ_FOUND
  BT::PublisherZMQ publisher_zmq(tree_);
#endif

  // This logger prints state changes on console
  BT::StdCoutLogger logger_cout(tree_);
  printTreeRecursively(tree_.rootNode());

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BTOperator::on_activate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(LOGGER, "Activating");

  auto is_canceling = [this]() {
    RCLCPP_INFO(LOGGER, "Is canceling");
    return false;
  };

  auto on_loop = [&]() { RCLCPP_INFO(LOGGER, "On loop"); };

  if (!loadBehaviorTree(parameters_->default_bt_xml_filename))
  {
    RCLCPP_ERROR(LOGGER, "Error loading XML file: %s", parameters_->default_bt_xml_filename.c_str());
    return nav2_util::CallbackReturn::FAILURE;
  }

  // BT::StdCoutLogger logger_cout(tree_);

  action_server_->activate();

  // create bond connection
  createBond();

  // man2_behavior_tree::BtStatus rc =
  //     bt_->run_loop(&tree_, on_loop, is_canceling, std::chrono::milliseconds(1));

  // std::this_thread::sleep_for(2s);
  // while (tree_.rootNode()->status() != BT::NodeStatus::SUCCESS ||
  //        tree_.rootNode()->status() != BT::NodeStatus::FAILURE)
  // {
  //   rclcpp::sleep_for(std::chrono::milliseconds(1));
  // }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BTOperator::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating");

  action_server_->deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BTOperator::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(LOGGER, "Cleaning up");

  parameters_.reset();
  blackboard_.reset();
  action_server_.reset();
  bt_->haltAllActions(tree_.rootNode());
  bt_.reset();

  RCLCPP_INFO(LOGGER, "Completed Cleaning up");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BTOperator::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(LOGGER, "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

bool BTOperator::loadBehaviorTree(const std::string& bt_xml_filename)
{
  // Use previous BT if it is the existing one
  if (parameters_->current_bt_xml_filename == bt_xml_filename)
  {
    return true;
  }

  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);

  if (!xml_file.good())
  {
    RCLCPP_ERROR(LOGGER, "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return false;
  }

  auto xml_string =
      std::string(std::istreambuf_iterator<char>(xml_file), std::istreambuf_iterator<char>());

  RCLCPP_DEBUG(LOGGER, "Behavior Tree file: '%s'", bt_xml_filename.c_str());
  RCLCPP_DEBUG(LOGGER, "Behavior Tree XML: %s", xml_string.c_str());

  // Create the Behavior Tree from the XML input
  tree_ = bt_->createTreeFromText(xml_string, blackboard_);
  parameters_->current_bt_xml_filename = bt_xml_filename;
  parameters_->setParameter(shared_from_this(), parameters_->current_bt_xml_filename);
  return true;
}

void BTOperator::startApplication()
{
  auto is_canceling = [this]() {
    if (action_server_ == nullptr)
    {
      RCLCPP_DEBUG(get_logger(), "Action server unavailable. Canceling.");
      return true;
    }

    if (!action_server_->is_server_active())
    {
      RCLCPP_DEBUG(get_logger(), "Action server is inactive. Canceling.");
      return true;
    }

    return action_server_->is_cancel_requested();
  };

  auto bt_xml_filename = action_server_->get_current_goal()->behavior_tree_filename;

  // Empty id in request is default for backward compatibility
  if (bt_xml_filename.empty() || bt_xml_filename == "" || bt_xml_filename == "None")
  {
    bt_xml_filename = parameters_->default_bt_xml_filename;
  }

  RCLCPP_ERROR(get_logger(), "parameters_->default_bt_xml_filename: %s",
               parameters_->default_bt_xml_filename.c_str());

  if (!loadBehaviorTree(bt_xml_filename))
  {
    RCLCPP_ERROR(get_logger(), "BT file not found: %s. Current: %s, Navigation canceled",
                 bt_xml_filename.c_str(), parameters_->current_bt_xml_filename.c_str());
    action_server_->terminate_current();
    return;
  }

  std::shared_ptr<Action::Feedback> feedback_msg = std::make_shared<Action::Feedback>();

  auto on_loop = [&]() {
    if (action_server_->is_preempt_requested())
    {
      RCLCPP_INFO(get_logger(), "Received goal preemption request");
      action_server_->accept_pending_goal();
    }

    // action server feedback
    RCLCPP_INFO(LOGGER, "on loop");
    int recovery_count = 0;
    blackboard_->get<int>("number_recoveries", recovery_count);
    feedback_msg->number_of_recoveries = recovery_count;
    feedback_msg->execution_time = now() - start_time_;
    action_server_->publish_feedback(feedback_msg);
  };

  // Execute the BT that was previously created in the configure step
  BT::StdCoutLogger logger_cout(tree_);
  nav2_behavior_tree::BtStatus rc = bt_->run(&tree_, on_loop, is_canceling);
  // Make sure that the Bt is not in a running state from a previous execution
  // note: if all the ControlNodes are implemented correctly, this is not needed.
  bt_->haltAllActions(tree_.rootNode());

  switch (rc)
  {
    case nav2_behavior_tree::BtStatus::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      action_server_->succeeded_current();
      break;

    case nav2_behavior_tree::BtStatus::FAILED:
      RCLCPP_ERROR(get_logger(), "Navigation failed");
      action_server_->terminate_current();
      break;

    case nav2_behavior_tree::BtStatus::CANCELED:
      RCLCPP_INFO(get_logger(), "Navigation canceled");
      action_server_->terminate_all();
      break;
  }
}

}  // namespace man2_bt_operator
