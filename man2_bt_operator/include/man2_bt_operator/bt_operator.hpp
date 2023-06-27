#ifndef MAN2_BT_OPERATOR__BT_OPERATOR_HPP_
#define MAN2_BT_OPERATOR__BT_OPERATOR_HPP_
#define STRINGIFY(x) #x

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
// #include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "man2_behavior_tree/behavior_tree_engine.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "man2_msgs/action/run_application.hpp"
#include "nav2_util/simple_action_server.hpp"

#ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

namespace man2_bt_operator
{
class BTOperator : public nav2_util::LifecycleNode
{
  /**
   * @brief Declare and load ros parameters
   *
   */
  struct RosParameters
  {
    std::vector<std::string> plugin_lib_names;
    std::string default_bt_xml_filename;
    std::string current_bt_xml_filename;
    bool print_bt_status;

    std::string ns = "";

    void declareRosParameters(const nav2_util::LifecycleNode::SharedPtr& node)
    {
      node->declare_parameter(ns + "current_bt_xml_filename", rclcpp::PARAMETER_STRING);
      node->declare_parameter(ns + "default_bt_xml_filename", rclcpp::PARAMETER_STRING);
      node->declare_parameter(ns + "plugin_lib_names", rclcpp::PARAMETER_STRING_ARRAY);
      node->declare_parameter(ns + "print_bt_status", rclcpp::PARAMETER_BOOL);
    }

    void loadRosParameters(const nav2_util::LifecycleNode::SharedPtr& node)
    {
      node->get_parameter_or(ns + "plugin_lib_names", plugin_lib_names, std::vector<std::string>{});
      node->get_parameter_or(ns + "default_bt_xml_filename", default_bt_xml_filename,
                             ament_index_cpp::get_package_share_directory("man2_bt_operator") +
                                 std::string("/tree/default_bt_xml_filename.xml"));
      node->get_parameter_or(ns + "print_bt_status", print_bt_status, false);

      RCLCPP_INFO(node->get_logger(),
                  "loadRosParameters: plugin_lib_names: %s, default_bt_xml_filename: %s",
                  rclcpp::Parameter("plugin_lib_names").value_to_string().c_str(),
                  default_bt_xml_filename.c_str());
    }

    bool setParameter(const nav2_util::LifecycleNode::SharedPtr& node,
                      const std::string& parameter_name)
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
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& /*state*/) override;

  /**
   * @brief do something during lifecycle node activation
   *
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& /*state*/) override;

  /**
   * @brief do something during lifecycle node deactivation
   *
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& /*state*/) override;

  /**
   * @brief do something during lifecycle node cleaning up
   *
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& /*state*/) override;

  /**
   * @brief do something during lifecycle node shutdown
   *
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& /*state*/) override;

  /**
   * @brief load a behavior tree xml
   *
   * @param bt_xml_filename
   * @return true
   * @return false
   */
  bool loadBehaviorTree(const std::string& bt_xml_filename);

  using Action = man2_msgs::action::RunApplication;

  using ActionServer = nav2_util::SimpleActionServer<Action>;

  // Our action server implements the NavigateToPose action
  std::unique_ptr<ActionServer> action_server_;

  void startApplication();

  // The wrapper class for the BT functionality
  std::unique_ptr<man2_behavior_tree::ManipulationBehaviorTreeEngine> bt_;

  BT::Tree tree_;

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;

  std::shared_ptr<RosParameters> parameters_;

  rclcpp::Time start_time_;
};

}  // namespace man2_bt_operator
#endif  // MAN_BEHAVIOR_TREE__BT_OPERATOR_HPP_
