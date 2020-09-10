#ifndef MAN_BEHAVIOR_TREE_NODES_SET_FLAG_TASK_
#define MAN_BEHAVIOR_TREE_NODES_SET_FLAG_TASK_

#include <string>
#include <memory>

#include "ros/ros.h"
#include "behaviortree_cpp_v3/condition_node.h"



namespace man_behavior_tree_nodes
{

class SetFlagTask : public BT::ConditionNode
{
public:
  SetFlagTask(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
  : BT::ConditionNode(condition_name, conf), initialized_(false)
  {
  }

  SetFlagTask() = delete;

  ~SetFlagTask()
  {
    cleanup();
  }

  BT::NodeStatus tick() override
  {
    if (!initialized_) {
      initialize();
    }

    if (isSetFlagTask()) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  void initialize()
  {  
    getInput("task_name", task_name_);

    getInput("success", task_result_);
    initialized_ = true;
  }

  bool isSetFlagTask()
  {
    param_bool_.clear();
    config().blackboard->get<std::map<std::string, bool>>("param_bool", param_bool_);
    for(auto it = param_bool_.cbegin(); it != param_bool_.cend(); ++it)
    {
        std::cout <<"old task param"<< it->first << " " << it->second << "\n";
    }    
    if(!param_bool_.find(task_name_)->first.empty())
        {
            param_bool_.at(task_name_) = task_result_;
            config().blackboard->set<std::map<std::string, bool>>("param_bool", param_bool_);

            for(auto it = param_bool_.cbegin(); it != param_bool_.cend(); ++it)
            {
                std::cout <<"new task param"<< it->first << " " << it->second<< "\n";
            }    

            return true;
        }
        return false;
  
  }
    

  static BT::PortsList providedPorts()
  {
    return {
            BT::InputPort<std::string>("task_name", "task name will be set"),
            BT::InputPort<bool>("success", "if task success"),
            };
  }

protected:
  void cleanup()
  {
  }

private:
  std::map<std::string, bool> param_bool_;
  std::string task_name_;
  bool task_result_;
  bool initialized_;
  double goal_reached_tol_;
};

}   //namespace

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<man_behavior_tree_nodes::SetFlagTask>("SetFlagTask");
}

#endif  // 