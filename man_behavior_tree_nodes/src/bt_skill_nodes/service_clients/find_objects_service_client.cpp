#include <string>
#include <memory>

#include "man_behavior_tree_nodes/bt_skill_nodes/service_clients/find_objects_service_client.hpp"

namespace man_behavior_tree_nodes
{

FindObjectsServiceClient::FindObjectsServiceClient(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
: btServiceClient<man_msgs::GetMarkerPose>(xml_tag_name, service_name, conf)
{
}

void FindObjectsServiceClient::on_tick()
{
  if(!getInput("object_id", service_.request.object_id)){
     ROS_ERROR_STREAM_NAMED(service_client_name_, service_client_name_ << ": Please provide object_id");
  }
  
  if(!getInput("container_a_id", service_.request.container_a_id)){
    ROS_ERROR_STREAM_NAMED(service_client_name_, service_client_name_ << ": Please provide container_a_id");
  }

  if(!getInput("container_b_id", service_.request.container_b_id)){
    ROS_ERROR_STREAM_NAMED(service_client_name_, service_client_name_ << ": Please provide container_b_id");
  }
} 

BT::NodeStatus FindObjectsServiceClient::on_success()
{
    setOutput("marker", service_.response.object);
    setOutput("container", service_.response.container);
    return BT::NodeStatus::SUCCESS;
}
} //namespace

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::FindObjectsServiceClient>(
        name, "skill_find_objects_server", config);
    };

  factory.registerBuilder<man_behavior_tree_nodes::FindObjectsServiceClient>(
    "FindObjects", builder);
}