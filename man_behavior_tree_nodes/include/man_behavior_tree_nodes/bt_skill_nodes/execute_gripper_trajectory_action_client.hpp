#ifndef MAN_BEHAVIOR_TREE_NODES_EXECUTE_GRIPPER_TRAJECTORY_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_EXECUTE_GRIPPER_TRAJECTORY_CLIENT_


#include "man_behavior_tree_nodes/bt_action_client.hpp"
#include "man_msgs/ExecuteGripperTrajectoryAction.h"

namespace man_behavior_tree_nodes
{
class ExecuteGripperTrajectoryActionClient : public btActionClient<man_msgs::ExecuteGripperTrajectoryAction, 
                                                        man_msgs::ExecuteGripperTrajectoryGoal,
                                                        man_msgs::ExecuteGripperTrajectoryResultConstPtr>
{
public:
    ExecuteGripperTrajectoryActionClient(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf,
        float time_for_wait);

    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<std::string>("action_name", ""),      
            BT::InputPort<std::string>("step", ""),        
            BT::OutputPort<int>("result", ""), 
        });
    }

private:
    bool first_time_{true};
    int success_{0};
    std::string step_;
    std::map<std::string, std::string> param_string_;


};
} // namespace

#endif