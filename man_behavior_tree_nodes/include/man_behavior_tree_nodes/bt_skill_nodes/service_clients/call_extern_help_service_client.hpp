#ifndef MAN_BEHAVIOR_TREE_NODES_CALL_EXTERN_HELP_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_CALL_EXTERN_HELP_CLIENT_

#include "man_msgs/Help.h"
#include "man_behavior_tree_nodes/bt_service_client.hpp"

namespace man_behavior_tree_nodes
{
class CallExternHelpServiceClient : public btServiceClient<man_msgs::Help>
{
    public:
    CallExternHelpServiceClient(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {

        });
    }

private:
    bool first_time_{true};

};
} // namespace

#endif