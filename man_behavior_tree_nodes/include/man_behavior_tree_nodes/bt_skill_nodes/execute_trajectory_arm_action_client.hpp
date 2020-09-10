#ifndef MAN_BEHAVIOR_TREE_NODES_ARM_EXECUTE_TRAJECTORY_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_ARM_EXECUTE_TRAJECTORY_CLIENT_


#include "man_behavior_tree_nodes/bt_action_client.hpp"
#include "man_msgs/ExecuteTrajectorySkillAction.h"

namespace man_behavior_tree_nodes
{
class ExecuteTrajectoryActionClient : public btActionClient<man_msgs::ExecuteTrajectorySkillAction, 
                                                        man_msgs::ExecuteTrajectorySkillGoal,
                                                        man_msgs::ExecuteTrajectorySkillResultConstPtr>
{
public:
    ExecuteTrajectoryActionClient(
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
            BT::InputPort<man_msgs::Plan>("plan", ""),             
            BT::OutputPort<int>("result", ""), 
            // BT::OutputPort<moveit_msgs::RobotState>("trajectory_start", ""),
            // BT::OutputPort<moveit_msgs::RobotTrajectory>("trajectory", ""),
            // BT::OutputPort<std::string>("group_name", ""),
            // BT::OutputPort<float>("planning_time", ""),


        });
    }

private:
    bool first_time_{true};
    man_msgs::Plan plan_;
    int success_{0};

};
} // namespace

#endif