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
  service_client_name_(service_name+"_client")
  {
    // ** share nodehandle in blackboard
    nh_ = config().blackboard->get<ros::NodeHandle>("node_handle");

    // Get the required items from the blackboard
    // server_timeout_ =
    //   config().blackboard->get<std::chrono::milliseconds>("server_timeout");
    // getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);

    // create the service client for this BT service
    std::string remapped_server_name;
    if (getInput("service_name", remapped_server_name)) 
    {
        service_name_ = remapped_server_name;
        ROS_INFO( "get service name: \"%s\"", service_name_.c_str());
    }

    service_client_ = pnh_.serviceClient<ServiceT>(service_name_);

    // Make a request for the service without parameter
    // request_ =  std::make_shared<typename ServiceT::Request>();

    // Make sure the server is actually there before continuing
    ROS_INFO( "Waiting for \"%s\" service", service_name_.c_str());
    if (ros::service::waitForService(service_name_))
    {
      ROS_INFO( "service: \"%s\" available", service_name_.c_str());
    }

    ROS_INFO("\"%s\"  initialized", service_client_name_.c_str());

    // auto success_ = service_client_.call(service_);

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
    ROS_INFO("\"%s\" call service", service_client_name_.c_str());
    // sauto success_ = service_client_.call(service_, request_, response_);
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
    ROS_INFO_STREAM_NAMED("bt_service_client", service_client_name_.c_str() << "Check result");
    if (success == true) {
      ROS_INFO_STREAM_NAMED("bt_service_client", service_client_name_.c_str()<< " call" << service_name_.c_str() << " succeeded");
      return on_success();
    } 
    ROS_INFO("service call failed");
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

    // typename ServiceT::Request request_;
    // typename ServiceT::Response response_;
    // std::shared_ptr<typename ServiceT::Request> request_;
    // std::shared_ptr<typename ServiceT::Response> response_;

    // typename ServiceT::Request request_;
    // typename ServiceT::Result result_;

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
