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
    const std::string & service_client_name,
    const BT::NodeConfiguration & conf)
  : BT::SyncActionNode(service_client_name, conf), service_client_name_(service_client_name)
  {
    // ** share nodehandle in blackboard
    pnh_ = config().blackboard->get<ros::NodeHandle>("private_node_handle");

    // Get the required items from the blackboard
    // server_timeout_ =
    //   config().blackboard->get<std::chrono::milliseconds>("server_timeout");
    // getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // create the service client for this BT service
    getInput("service_name", service_name_);
    service_client_ = pnh_.serviceClient<ServiceT>(service_name_);

    // Make a request for the service without parameter
    request_ = typename ServiceT::Request();

    // Make sure the server is actually there before continuing
    ROS_INFO( "Waiting for \"%s\" service", service_name_.c_str());
    ros::service::waitForService(service_name_);

    ROS_INFO("\"%s\" BtServiceNode initialized",
    service_client_name_.c_str());

  }

  btServiceNode() = delete;

  virtual ~btServiceNode()
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
    auto success = service_client_.call(service_name_, request_, result_);
    return check_result(success);
  }

  // Fill in service request with information if necessary
  virtual void on_tick()
  {
  }

  // Check the future and decide the status of Behaviortree
  virtual BT::NodeStatus check_result(bool success)
  {
    if (success == true) {
      return BT::NodeStatus::SUCCESS;
    } 
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
    typename ServiceT service_;

    typename ServiceT::Request request_;
    typename ServiceT::Result result_;

    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;

    // The timeout value while to use in the tick loop while waiting for
    // a result from the server
    std::chrono::milliseconds server_timeout_;
};

}  // namespace nav2_behavior_tree

#endif  // MAN_BEHAVIOR_TREE_NODES__BT_SERVICE_CLIENT_HPP_
