#include <string>
#include <memory>

#include "man_behavior_tree_nodes/bt_skill_nodes/update_param_service_client.hpp"

namespace man_behavior_tree_nodes
{

UpdateParamServiceClient::UpdateParamServiceClient(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
: btServiceClient<man_msgs::UpdateParam>(xml_tag_name, service_name, conf)
{
}

void UpdateParamServiceClient::on_tick()
{
  getInput("topic", service_.request.topic);
  
  getInput("data_type", service_.request.data_type);
  // ROS_INFO( "request: \"%s\", \"%s\"", request_->topic.c_str(), request_->data_type.c_str());

}  // namespace man_behavior_tree_nodes

BT::NodeStatus UpdateParamServiceClient::on_success()
{
    param_float_.clear();
    param_string_.clear();
    param_int_.clear();
    param_bool_.clear();

    for (unsigned i=0; i<service_.response.label.size(); i++)
    {
      if (service_.request.data_type.compare("double") == 0)
      {
        param_float_.insert({service_.response.label.at(i), service_.response.value_double.at(i)});
        }

        if (service_.request.data_type.compare("string") == 0)
      {
        param_string_.insert({service_.response.label.at(i), service_.response.value_string.at(i)});
        }

      if (service_.request.data_type.compare("bool") == 0)
      {
        param_bool_.insert({service_.response.label.at(i), service_.response.value_int.at(i)});
        }

      if (service_.request.data_type.compare("int") == 0)
      {
        param_int_.insert({service_.response.label.at(i), service_.response.value_int.at(i)});
        }       
        
    }


    if(param_string_.size() != 0)
    {
      // for(auto it = param_string_.cbegin(); it != param_string_.cend(); ++it)
      // {
      //     std::cout << it->first << " " << it->second << " "  << "\n";
      // }
      config().blackboard->set<std::map<std::string, std::string>>("param_string", param_string_);
      return BT::NodeStatus::SUCCESS;
    }

    if(param_float_.size() != 0)
    {
      for(auto it = param_float_.cbegin(); it != param_float_.cend(); ++it)
      {
          std::cout << it->first << " " << it->second << " "  << "\n";
      }
      config().blackboard->set<std::map<std::string, float>>("param_float", param_float_);

      return BT::NodeStatus::SUCCESS;
    }

    if(param_bool_.size() != 0)
    {
      // for(auto it = param_bool_.cbegin(); it != param_bool_.cend(); ++it)
      // {
      //     std::cout << it->first << " " << it->second << " "  << "\n";
      // }
      config().blackboard->set<std::map<std::string, bool>>("param_bool", param_bool_);
      return BT::NodeStatus::SUCCESS;      
    }

    if(param_int_.size() != 0)
    {
      // for(auto it = param_int_.cbegin(); it != param_int_.cend(); ++it)
      // {
      //     std::cout << it->first << " " << it->second << " "  << "\n";
      // }
      config().blackboard->set<std::map<std::string, int8_t>>("param_int", param_int_);
      return BT::NodeStatus::SUCCESS;
    }

}
} //namespace


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::UpdateParamServiceClient>(
        name, "update_param", config);
    };

  factory.registerBuilder<man_behavior_tree_nodes::UpdateParamServiceClient>(
    "UpdateParameter", builder);
}