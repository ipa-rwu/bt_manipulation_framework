#include <string>
#include <memory>

#include "man_behavior_tree_nodes/bt_skill_nodes/set_param_service_client.hpp"

namespace man_behavior_tree_nodes
{

SetParamServiceClient::SetParamServiceClient(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
: btServiceClient<man_msgs::SetFlagTask>(xml_tag_name, service_name, conf)
{
}

void SetParamServiceClient::on_tick()
{
  getInput("param_topic", service_.request.param_topic);
  
  getInput("step", service_.request.task_name);

  if(!getInput("value", service_.request.param_float))
  {
    double param;
    config().blackboard->get<double>("recovery_arm_parameter", param);
    service_.request.param_float = (float)param;
  }
  // ROS_INFO( "request: \"%s\", \"%s\"", request_->topic.c_str(), request_->data_type.c_str());

}  // namespace man_behavior_tree_nodes

BT::NodeStatus SetParamServiceClient::on_success()
{
    // ROS_INFO("get result");
    param_float_.clear();


    if(service_.response.result)
    {
        
        config().blackboard->get<std::map<std::string, float>>("param_float", param_float_);
        if(!param_float_.find(service_.request.task_name)->first.empty())
        {
            param_float_.at(service_.request.task_name) = service_.request.param_float;
            config().blackboard->set<std::map<std::string, float>>("param_float", param_float_);
            std::cout << "update" <<std::endl;
            for(auto it = param_float_.cbegin(); it != param_float_.cend(); ++it)
           {
              std::cout << it->first << " " << it->second << " "  << "\n";
            }
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
      return std::make_unique<man_behavior_tree_nodes::SetParamServiceClient>(
        name, "set_parameter_server", config);
    };

  factory.registerBuilder<man_behavior_tree_nodes::SetParamServiceClient>(
    "SetParameter", builder);
}