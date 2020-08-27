#ifndef MAN_BT_OPERATOR__BT_OPERATOR_HPP_
#define MAN_BT_OPERATOR__BT_OPERATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <map>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "ros/ros.h"
#include "man_behavior_tree_nodes/bt_engine.hpp"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#ifdef ZMQ_FOUND
    #include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#endif

namespace man_bt_operator
{
class BT_Manipulator
{

public:
    BT_Manipulator(
    const ros::NodeHandle &private_node_handle,
    std::string namespace_param,
    std::string robot_namespace_param);

     ~BT_Manipulator();

    bool loadBehaviorTree(const std::string & bt_id);
protected:
    
    void initialize();
    void initialize_robot();

    std::unique_ptr<man_behavior_tree_nodes::BT_Engine> bt_;
    std::vector<std::string> plugin_lib_names_;

    BT::Tree tree_;
    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;
    std::map<std::string, float> param_float_;
    std::map<std::string, std::string> param_string_;
    std::map<std::string, int8_t> param_int_;
    std::map<std::string, bool> param_bool_;

    // The XML fiñe that cointains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    ros::NodeHandle pnh_;

    std::string bt_namespace_param_;
    std::string robot_namespace_param_;

    // get info from robot
    std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr kinematic_state_;
    std::string base_frame_;
    std::string group_name_gripper_;
    std::string group_name_arm_;
};


}
#endif // MAN_BEHAVIOR_TREE__BT_OPERATOR_HPP_