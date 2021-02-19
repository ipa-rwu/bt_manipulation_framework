#include "man_behavior_tree_nodes/bt_skill_nodes/action_clients/execute_gripper_io_action_client.hpp"

namespace man_behavior_tree_nodes
{

ExecuteGripperIOActionClient::ExecuteGripperIOActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait,
  const std::string & subscribe_topic_name)
: btActionClient<man_msgs::ExecuteGripperIOAction, 
                man_msgs::ExecuteGripperIOGoal,
                man_msgs::ExecuteGripperIOResultConstPtr,
                man_msgs::ExecuteGripperIOFeedbackConstPtr,
                EmptyClass>
                  (xml_tag_name, action_name, conf, time_for_wait, subscribe_topic_name)
{
}

void ExecuteGripperIOActionClient::on_tick(){
    if(!getInput("fun", goal_.fun)){
        goal_.fun = io_fun_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client" << ": Using default IO FUN: " << goal_.fun);
    if(!getInput("pin", goal_.pin)){
        goal_.pin = io_pin_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client" << ": Using default IO pin: " <<  goal_.pin);

    getInput("robot_settle_time", robot_settle_time_);
    
    if(!getInput("action", goal_.action)){
      if(!getInput("step", step_))
          throw BT::RuntimeError(action_name_+"_client" + ": Please provide gripper action or task step");
      else
      {
          config().blackboard->get<std::string>("current_step", step_); 
          config().blackboard->get<std::map<std::string, std::string>>("param_string", param_string_);
          goal_.action = param_string_.find(step_)->second;
      }
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client" << ": Using action: " << goal_.action );
    sleepSafeFor(robot_settle_time_);
}

void ExecuteGripperIOActionClient::sleepSafeFor(double duration)
{
  ros::Time start = ros::Time::now();
  while(ros::Time::now() - start <= ros::Duration(duration))
  {
    ros::spinOnce();
  }
}

BT::NodeStatus ExecuteGripperIOActionClient::on_success(){
    if(result_->success)
    {
       return BT::NodeStatus::SUCCESS;
    }
    else{
        return BT::NodeStatus::FAILURE;
    }
}

} //namespace

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  float time_for_wait = 5.0;
  BT::NodeBuilder builder =
    [time_for_wait](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::ExecuteGripperIOActionClient>(
        name, "skill_execute_gripper_io_server", config, time_for_wait, "");
    };

  factory.registerBuilder<man_behavior_tree_nodes::ExecuteGripperIOActionClient>(
    "ExecuteGripperIO", builder);
}