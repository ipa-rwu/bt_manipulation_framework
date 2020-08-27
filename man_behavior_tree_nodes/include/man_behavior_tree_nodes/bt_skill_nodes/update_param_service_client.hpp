#ifndef MAN_BEHAVIOR_TREE_NODES_UPDATE_PARAM_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_UPDATE_PARAM_CLIENT_

#include "man_msgs/UpdateParam.h"
#include "man_behavior_tree_nodes/bt_service_client.hpp"

namespace man_behavior_tree_nodes
{
class UpdateParamServiceClient : public btServiceClient<man_msgs::UpdateParam>
{
    public:
    UpdateParamServiceClient(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<std::string>("topic", "parameter topic"),
            BT::InputPort<std::string>("data_type", "parameter data type"),

        });
    }

private:
    bool first_time_{true};

    std::map<std::string, float> param_float_;
    std::map<std::string, std::string> param_string_;
    std::map<std::string, int8_t> param_int_;
    std::map<std::string, bool> param_bool_;

};
} // namespace

#endif