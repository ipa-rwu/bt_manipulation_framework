#ifndef MAN_BEHAVIOR_TREE_NODES_CHECK_TASK_RESULT_
#define MAN_BEHAVIOR_TREE_NODES_CHECK_TASK_RESULT_

#include <string>
#include <memory>
#include "ros/ros.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace man_behavior_tree_nodes
{

class CheckTaskResult : public BT::ConditionNode
{
public:
  CheckTaskResult(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf),
  condition_name_(condition_name){
  }

  CheckTaskResult() = delete;

  ~CheckTaskResult(){
    cleanup();
  }

  BT::NodeStatus tick() override
  {
    initialize();

    if (isTaskSuccess()){
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  void initialize(){
    config().blackboard->get<bool>("first_time", first_time_);

    if(!getInput("task_name", task_name_)){
      ROS_ERROR_STREAM_NAMED(condition_name_, condition_name_ << ": Please provide the name of task");
    }

    param_bool_.clear();
    config().blackboard->get<std::map<std::string, bool>>("param_bool", param_bool_);
  }

  bool isTaskSuccess(){
    if(first_time_ == true){ 
       return false;
    }
    
    if(first_time_ == false)
    {
      if (task_name_.compare("Help") == 0 && param_bool_.at("Help") == 1)
      {      
        return false;
      }

      if (task_name_.compare("Help") == 0 && param_bool_.at("Help") == 0)
      {
        return true;
      }

      if (param_bool_.at(task_name_) == 1){
        return true;
      }
      else
      {
        return false;
      }
    }
  }
    

  static BT::PortsList providedPorts()
  {
    return {
            BT::InputPort<std::string>("task_name", "task name will be set"),
            };
  }

protected:
  void cleanup()
  {
  }

private:
  std::map<std::string, bool> param_bool_;
  std::string task_name_;
  bool first_time_;
  std::string condition_name_;
};

}   //namespace

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<man_behavior_tree_nodes::CheckTaskResult>("CheckTaskResult");
}

#endif  // 