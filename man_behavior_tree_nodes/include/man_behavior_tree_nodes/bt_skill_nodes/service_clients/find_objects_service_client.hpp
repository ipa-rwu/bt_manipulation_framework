#ifndef MAN_BEHAVIOR_TREE_NODES_FIND_OBJECTS_SERVER_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_FIND_OBJECTS_SERVER_CLIENT_

#include "man_msgs/GetMarkerPose.h"
#include "man_behavior_tree_nodes/bt_service_client.hpp"

namespace man_behavior_tree_nodes
{
class FindObjectsServiceClient : public btServiceClient<man_msgs::GetMarkerPose>
{
    public:
    FindObjectsServiceClient(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<int>("object_id", "object_id"),
            BT::InputPort<int>("container_a_id", "container_a_id"),
            BT::InputPort<int>("container_b_id", "container_b_id"),
            BT::OutputPort<geometry_msgs::PoseStamped>("container", "posestamped for container"),
            BT::OutputPort<geometry_msgs::PoseStamped>("object", "posestamped for marker"),
        });
    }

private:
    bool first_time_{true};
    std::map<std::string, float> param_float_;
    std::map<std::string, bool> param_bool_;

};
} // namespace

#endif