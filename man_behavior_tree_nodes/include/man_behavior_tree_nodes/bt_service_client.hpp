#ifndef MAN_BEHAVIOR_TREE_NODES__BT_SERVICE_CLIENT_HPP_
#define MAN_BEHAVIOR_TREE_NODES__BT_SERVICE_CLIENT_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "ros/ros.h"

namespace man_behavior_tree_nodes
{
// input: "server_timeout", "service_name"
template<class ServiceT>
class btServiceClient : public BT::SyncActionNode
{
public:
  btServiceClient(
    const std::string & xml_tag_name,
    const std::string & service_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(xml_tag_name, conf), 
  service_name_(service_name)
  {
    // ** share nodehandle in blackboard
    pnh_ = config().blackboard->get<ros::NodeHandle>("node_handle");

    // create the service client for this BT service
    std::string remapped_server_name;
    if (getInput("service_name", remapped_server_name)) 
    {
      service_name_ = remapped_server_name;
      ROS_DEBUG_NAMED("BTService", "get service name: \"%s\"", service_name_.c_str());
    }

    service_client_ = pnh_.serviceClient<ServiceT>(service_name_);

    ROS_INFO_NAMED(service_name_.c_str(), "Waiting for service: \"%s\"", service_name_.c_str());
    if (ros::service::waitForService(service_name_))
    {
      ROS_INFO_NAMED(service_name_.c_str(), "service: \"%s\" available", service_name_.c_str());
    }

    service_client_name_ = service_name_ + "_client";
  }

  btServiceClient() = delete;

  virtual ~btServiceClient()
  {
  }

  // Any subclass of BtServiceNode that accepts parameters must provide a providedPorts method
  // and call providedBasicPorts in it.
  static BT::PortsList providedBasicPorts(BT::PortsList addition)
  {
    BT::PortsList basic = {
      BT::InputPort<std::string>("service_name", "please_set_service_name_in_BT_Node"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout")
    };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  // The main override required by a BT service
  BT::NodeStatus tick() override
  {
    on_tick();
    auto success_ = service_client_.call(service_);
    // get feedback from service server
    return check_result(success_);
  }

  // Fill in service request with information if necessary
  virtual void on_tick()
  {
  }

  virtual BT::NodeStatus on_success()
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Check the future and decide the status of Behaviortree
  virtual BT::NodeStatus check_result(bool success)
  {
    if (success == true) {
      return on_success();
    } 
    ROS_WARN_STREAM_NAMED(service_name_.c_str(), service_name_.c_str() << ": service call failed!!!");
    return BT::NodeStatus::FAILURE;
  }

  // An opportunity to do something after
  // a timeout waiting for a result that hasn't been received yet
  virtual void on_wait_for_result()
  {
  }

protected:
    std::string service_name_, service_client_name_;
    ros::ServiceClient service_client_;
    
    // declare service will be used
    ServiceT service_;

    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;

    // The timeout value while to use in the tick loop while waiting for
    // a result from the server
    std::chrono::milliseconds server_timeout_;

    bool success_;
};

}  // namespace nav2_behavior_tree

#endif  // MAN_BEHAVIOR_TREE_NODES__BT_SERVICE_CLIENT_HPP_
