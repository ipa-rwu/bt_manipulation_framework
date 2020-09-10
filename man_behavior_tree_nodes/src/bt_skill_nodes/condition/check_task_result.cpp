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
  : BT::ConditionNode(condition_name, conf), initialized_(false)
  {
  }

  CheckTaskResult() = delete;

  ~CheckTaskResult()
  {
    cleanup();
  }

  BT::NodeStatus tick() override
  {
    initialized_ = false;
    if (!initialized_) {
      initialize();
    }

    if (isTaskSuccess()) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  void initialize()
  {

    std::cout << "initialized_: start"<< std::endl;
    getInput("task_name", task_name_);
    std::cout << "check task name:    "<< task_name_ << std::endl;
    config().blackboard->get<bool>("first_time", first_time_);
    std::cout << "first_time:    "<< first_time_<< std::endl;

    param_bool_.clear();
    config().blackboard->get<std::map<std::string, bool>>("param_bool", param_bool_);
    initialized_ = true;
    std::cout << "initialized_: true"<< std::endl;
  }

  bool isTaskSuccess()
  {
    std::cout << "isTaskSuccess Function"<< std::endl;
    if(first_time_ == 1)
    { 
       return false;
    }
    
    if(first_time_ == 0)
    {
     std::cout <<  "  value: " << param_bool_.at(task_name_) <<std::endl;


    if (task_name_.compare("Help") == 0 && param_bool_.at(task_name_) == 1)
    {
      std::cout << "!!!!!!"<<std::endl;
      return false;
    }

    if (task_name_.compare("Help") == 0 && param_bool_.at(task_name_) == 0)
    {
      std::cout << "######" <<std::endl;
      return true;
    }

    if (param_bool_.at(task_name_) == 1)
        return true;
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
  bool initialized_;
  bool first_time_;
};

}   //namespace

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<man_behavior_tree_nodes::CheckTaskResult>("CheckTaskResult");
}

#endif  // 