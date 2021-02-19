#include <string>
#include <memory>

#include "man_behavior_tree_nodes/bt_skill_nodes/service_clients/set_param_service_client.hpp"

namespace man_behavior_tree_nodes
{

SetParamServiceClient::SetParamServiceClient(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
: btServiceClient<man_msgs::SetParameter>(xml_tag_name, service_name, conf)
{
}

void SetParamServiceClient::on_tick()
{
  if(!getInput("param_topic", service_.request.param_topic)){
    // ROS_ERROR_STREAM_NAMED(service_client_name_, service_client_name_ << ": Please provide dynamic parameter topic ");
  }
  
  if(!getInput("label", service_.request.label)){
    ROS_ERROR_STREAM_NAMED(service_client_name_, service_client_name_ << ": Please provide parameter name");
  };

  if(!getInput("data_type", service_.request.data_type)){
    ROS_ERROR_STREAM_NAMED(service_client_name_, service_client_name_ << ": Please provide data type");
  }
  else
  {
    if (service_.request.data_type.compare("double") == 0){
      double param;
      if(!getInput("value", param))
      {
        config().blackboard->get<double>("recovery_arm_parameter", param);
        service_.request.param_float = (float)param;
      }
      else{
        service_.request.param_float = param;
      }
    }
    if (service_.request.data_type.compare("bool") == 0){
      bool param;
      if(!getInput("value", param))
      {
        ROS_ERROR_STREAM_NAMED(service_client_name_, service_client_name_ << ": Please provide param value");
      }
      else{
        service_.request.param_bool = param;
      }
    }
  }
} 

BT::NodeStatus SetParamServiceClient::on_success()
{
    if(service_.response.result)
    {
      if(service_.request.data_type.compare("double") == 0){
        param_float_.clear();
        config().blackboard->get<std::map<std::string, float>>("param_float", param_float_);
        if(!param_float_.find(service_.request.label)->first.empty())
        {
          param_float_.at(service_.request.label) = service_.request.param_float;
          config().blackboard->set<std::map<std::string, float>>("param_float", param_float_);
          for(auto it = param_float_.cbegin(); it != param_float_.cend(); ++it)
         {
           ROS_DEBUG_STREAM_NAMED(service_client_name_, service_client_name_ << ": "<< it->first << " " << it->second << " "  << "\n");
          }
          return BT::NodeStatus::SUCCESS;
        }
      }

      if(service_.request.data_type.compare("bool") == 0){
        param_bool_.clear();
        config().blackboard->get<std::map<std::string, bool>>("param_bool", param_bool_);
        if(!param_bool_.find(service_.request.label)->first.empty())
        {
          param_bool_.at(service_.request.label) = service_.request.param_bool;
          config().blackboard->set<std::map<std::string, bool>>("param_bool", param_bool_);
          for(auto it = param_bool_.cbegin(); it != param_bool_.cend(); ++it)
         {
           ROS_DEBUG_STREAM_NAMED(service_client_name_, service_client_name_ << ": "<< it->first << " " << it->second << " "  << "\n");
          }
          return BT::NodeStatus::SUCCESS;
        }
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
        name, "skill_set_parameter_server", config);
    };

  factory.registerBuilder<man_behavior_tree_nodes::SetParamServiceClient>(
    "SetParameter", builder);
}