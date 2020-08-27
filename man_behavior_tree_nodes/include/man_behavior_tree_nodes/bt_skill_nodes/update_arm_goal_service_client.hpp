#ifndef MAN_BEHAVIOR_TREE_NODES_UPDATE_ARM_GOAL_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_UPDATE_ARM_GOAL_CLIENT_

#include "man_msgs/UpdateArmGoal.h"
#include "man_behavior_tree_nodes/bt_service_client.hpp"

namespace man_behavior_tree_nodes
{
class UpdateArmGoalServiceClient : public btServiceClient<man_msgs::UpdateArmGoal>
{
    public:
    UpdateArmGoalServiceClient(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<std::string>("step", "step for goal"),
            BT::InputPort<std::string>("goal_frame_id", "frame id of goal"),
            BT::InputPort<geometry_msgs::PoseStamped>("target", "target for task to reach"),
            
            BT::OutputPort<geometry_msgs::PoseStamped>("goal", "goal for arm"),

        });
    }

private:
    bool first_time_{true};

    std::string step_;
    std::string goal_frame_id_;
    geometry_msgs::PoseStamped target_;
    float param_;

    std::map<std::string, float> param_float_;

};
} // namespace

#endif