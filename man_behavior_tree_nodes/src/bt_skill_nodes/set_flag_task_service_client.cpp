#include <string>
#include <memory>

#include "man_behavior_tree_nodes/bt_skill_nodes/set_flag_task_service_client.hpp"

namespace man_behavior_tree_nodes
{

SetFlagTaskServiceClient::SetFlagTaskServiceClient(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
: btServiceClient<man_msgs::SetFlagTask>(xml_tag_name, service_name, conf)
{
}

void SetFlagTaskServiceClient::on_tick()
{
  getInput("param_topic", service_.request.param_topic);
  
  getInput("task_name", service_.request.task_name);

  getInput("success", service_.request.success);
  // ROS_INFO( "request: \"%s\", \"%s\"", request_->topic.c_str(), request_->data_type.c_str());

}  // namespace man_behavior_tree_nodes

BT::NodeStatus SetFlagTaskServiceClient::on_success()
{
    // ROS_INFO("get result");
    param_bool_.clear();


    if(service_.response.result)
    {
        
        config().blackboard->get<std::map<std::string, bool>>("param_bool", param_bool_);
        if(!param_bool_.find(service_.request.task_name)->first.empty())
        {
            param_bool_.at(service_.request.task_name) = service_.request.success;
            config().blackboard->set<std::map<std::string, bool>>("param_bool", param_bool_);
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;

}
} //namespace


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::SetFlagTaskServiceClient>(
        name, "set_flag_for_task", config);
    };

  factory.registerBuilder<man_behavior_tree_nodes::SetFlagTaskServiceClient>(
    "SetFlagTaskClient", builder);
}