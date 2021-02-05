#ifndef MAN_BEHAVIOR_TREE_NODES_SET_PARAM_SERVER_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_SET_PARAM_SERVER_CLIENT_

#include "man_msgs/SetFlagTask.h"
#include "man_behavior_tree_nodes/bt_service_client.hpp"

namespace man_behavior_tree_nodes
{
class SetParamServiceClient : public btServiceClient<man_msgs::SetFlagTask>
{
    public:
    SetParamServiceClient(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<std::string>("param_topic", "parameter topic"),
            BT::InputPort<std::string>("step", "task name will be set"),
            BT::InputPort<float>("value", "if task success"),

        });
    }

private:
    bool first_time_{true};
    std::map<std::string, float> param_float_;

};
} // namespace

#endif