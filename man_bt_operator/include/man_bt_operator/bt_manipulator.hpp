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

namespace man_bt_operator
{
class BT_Manipulator
{

public:
    BT_Manipulator(
    const ros::NodeHandle &private_node_handle,
    std::string namespace_param);

     ~BT_Manipulator();

    bool loadBehaviorTree(const std::string & bt_id);
protected:
    
    void initialize();

    std::unique_ptr<man_behavior_tree_nodes::BT_Engine> bt_;
    std::vector<std::string> plugin_lib_names_;

    BT::Tree tree_;
    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;
    std::map<std::string, _Float32> param_float_;
    // The XML fiñe that cointains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    ros::NodeHandle pnh_;

    std::string namespace_param_;

};


}
#endif // MAN_BEHAVIOR_TREE__BT_OPERATOR_HPP_