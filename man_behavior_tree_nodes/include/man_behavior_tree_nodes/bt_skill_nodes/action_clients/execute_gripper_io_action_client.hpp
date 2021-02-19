#ifndef MAN_BEHAVIOR_TREE_NODES_EXECUTE_GRIPPER_IO_ACTION_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_EXECUTE_GRIPPER_IO_ACTION_CLIENT_

#include "man_msgs/ExecuteGripperIOAction.h"
#include "man_behavior_tree_nodes/bt_action_client.hpp"

namespace man_behavior_tree_nodes
{
class EmptyClass{};
class ExecuteGripperIOActionClient : public btActionClient<man_msgs::ExecuteGripperIOAction, 
                                                        man_msgs::ExecuteGripperIOGoal,
                                                        man_msgs::ExecuteGripperIOResultConstPtr,
                                                        man_msgs::ExecuteGripperIOFeedbackConstPtr,
                                                        EmptyClass>
{
    public:
    ExecuteGripperIOActionClient(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf,
        float time_for_wait,
        const std::string & subscribe_topic_name);
    
    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<int8_t>("fun", "io service fun leavel"),
            BT::InputPort<int8_t>("pin", "io pin"),
            BT::InputPort<std::string>("action", "gripper open or close"),
            BT::InputPort<std::string>("step", "task step"),
            BT::InputPort<double>("robot_settle_time", "time for waiting gripper settling"),
        });
    }

    void sleepSafeFor(double duration);

private:
    int io_fun_{1};
    int io_pin_;
    std::string action_;
    std::string step_;
    std::map<std::string, std::string> param_string_;
    double robot_settle_time_{2};
};
} // namespace

#endif //MAN_BEHAVIOR_TREE_NODES_EXECUTE_GRIPPER_IO_ACTION_CLIENT_