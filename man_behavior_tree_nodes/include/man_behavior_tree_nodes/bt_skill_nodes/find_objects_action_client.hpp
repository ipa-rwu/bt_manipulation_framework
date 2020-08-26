
#ifndef MAN_BEHAVIOR_TREE_NODES_FIND_OBJECTS_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_FIND_OBJECTS_CLIENT_

#include "man_behavior_tree_nodes/bt_action_client.hpp"
#include "man_msgs/FindObjectsAction.h"


namespace man_behavior_tree_nodes
{
class FindObjectsActionClient : public btActionClient<man_msgs::FindObjectsAction, 
                                                        man_msgs::FindObjectsGoal,
                                                        man_msgs::FindObjectsResultConstPtr>
{
public:
    FindObjectsActionClient(
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
            BT::OutputPort<geometry_msgs::PoseStamped>("container", "posestamped for container"),
            BT::OutputPort<geometry_msgs::PoseStamped>("marker", "posestamped for marker"),
            BT::InputPort<geometry_msgs::PoseStamped>("container_A", "posestamped for container A"),
            BT::InputPort<geometry_msgs::PoseStamped>("container_B", "posestamped for container B"),
            BT::InputPort<int32_t>("marker_id", "the id of marker"),
            BT::InputPort<std::string>("frame_id", "frame_id for marker and containers"),
        });
    }

private:
  bool first_time_{true};

};
} // namespace

#endif