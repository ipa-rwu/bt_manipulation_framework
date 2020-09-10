#include <string>
#include <memory>

#include "man_behavior_tree_nodes/bt_skill_nodes/call_extern_help_service_client.hpp"

namespace man_behavior_tree_nodes
{

CallExternHelpServiceClient::CallExternHelpServiceClient(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
: btServiceClient<man_msgs::Help>(xml_tag_name, service_name, conf)
{
}

void CallExternHelpServiceClient::on_tick()
{
    config().blackboard->get<bool>("first_time", first_time_);
    std::cout << "CallExternHelpService first_time:    "<< first_time_ << std::endl;;

    if(first_time_ == false)
        service_.request.needhelp = true;
  // ROS_INFO( "request: \"%s\", \"%s\"", request_->topic.c_str(), request_->data_type.c_str());

}  // namespace man_behavior_tree_nodes

BT::NodeStatus CallExternHelpServiceClient::on_success()
{
    if(first_time_)
    {
        first_time_ = false;
        config().blackboard->set<bool>("first_time", first_time_);
        return BT::NodeStatus::FAILURE;
    }
       
    else
    {
        if(service_.response.finish)
            return BT::NodeStatus::SUCCESS;
        else
        {
            return BT::NodeStatus::FAILURE;
        }
        
    }
}
} //namespace


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::CallExternHelpServiceClient>(
        name, "call_extern_help", config);
    };

  factory.registerBuilder<man_behavior_tree_nodes::CallExternHelpServiceClient>(
    "CallExternHelp", builder);
}