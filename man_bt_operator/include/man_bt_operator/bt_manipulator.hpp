#ifndef MAN_BT_OPERATOR__BT_OPERATOR_HPP_
#define MAN_BT_OPERATOR__BT_OPERATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "man_behavior_tree/bt_engine.hpp"

namespace man_bt_operator
{
class BT_Manipulator
{

public:
    BT_Manipulator();

    ~BT_Manipulator():


protected:
    bool loadBehaviorTree(const std::string & bt_id);


    std::unique_ptr<man_behavior_tree::BT_Engine> bt_;
    std::vector<std::string> plugin_lib_names_;

    BT::Tree tree_;
    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;
    // The XML fiñe that cointains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;
private:
    /* data */
};


}
#endif // MAN_BEHAVIOR_TREE__BT_OPERATOR_HPP_