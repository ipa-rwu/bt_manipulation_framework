#include "man_behavior_tree_nodes/bt_skill_nodes/action_clients/compute_path_action_client.hpp"

namespace man_behavior_tree_nodes
{

ComputePathActionClient::ComputePathActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait,
  const std::string & subscribe_topic_name)
: btActionClient<man_msgs::ComputePathSkillAction,
                  man_msgs::ComputePathSkillGoal,
                  man_msgs::ComputePathSkillResultConstPtr,
                  man_msgs::ComputePathSkillFeedbackConstPtr,
                  EmptyClass>
                  (xml_tag_name, action_name, conf, time_for_wait, subscribe_topic_name)
{
}

void ComputePathActionClient::on_tick()
{
    config().blackboard->get<std::string>("group_name_arm", goal_.group_name);
    config().blackboard->get<std::string>("end_effector", goal_.end_effector);
    if(goal_.group_name.empty()){
      throw BT::RuntimeError((action_name_),"_client: Did not get group_name from blackboard, please check configuration file");
    }
    if(goal_.end_effector.empty()){
      throw BT::RuntimeError((action_name_),"_client: Did not get end_effector name from blackboard, please check configuration file");
    }

    if(!getInput("target_type", goal_.target_type))
       throw BT::RuntimeError((action_name_),"_client: missing required input [target_type]");
    else{
      if (goal_.target_type.compare("Pose") == 0 || goal_.target_type.compare("Cartesian") == 0){
        if(!getInput("goal", goal_.goal)){
          throw BT::RuntimeError((action_name_),"_client: missing required input [goal]");
        }
      }
      if (goal_.target_type.compare("Name") == 0){
        if(!getInput("goal_name", goal_.named_goal)){
          throw BT::RuntimeError((action_name_),"_client: missing required input [goal_name]");
        }
      }
    }

    if(!getInput("replan_times", goal_.num_planning_attempts)){
      goal_.num_planning_attempts = replan_times_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: num_planning_attempts: "<<goal_.num_planning_attempts);

    if(!getInput("planner_id", goal_.planner_id)){
      goal_.planner_id = planner_id_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: planner_id: "<<goal_.planner_id);

    if(!getInput("allowed_planning_time", goal_.allowed_planning_time)){
      goal_.allowed_planning_time = allowed_planning_time_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: allowed_planning_time: "<<goal_.allowed_planning_time);

    if(!getInput("max_acceleration_scaling_factor", goal_.max_acceleration_scaling_factor)){
      goal_.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: max_acceleration_scaling_factor: "<<goal_.max_acceleration_scaling_factor);

    if(!getInput("max_velocity_scaling_factor", goal_.max_velocity_scaling_factor)){
      goal_.max_velocity_scaling_factor = max_velocity_scaling_factor_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: max_velocity_scaling_factor: "<<goal_.max_velocity_scaling_factor);


    if(!getInput("eef_step", goal_.eef_step)){
      goal_.eef_step = eef_step_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: eef_step: "<<goal_.eef_step);

    if(!getInput("jump_threshold", goal_.jump_threshold)){
      goal_.jump_threshold = jump_threshold_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: jump_threshold: "<<goal_.jump_threshold);

    if(!getInput("position_tolerances", goal_.position_tolerances)){
      goal_.position_tolerances = position_tolerances_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: position_tolerances: "<<goal_.position_tolerances.at(1));

    
    if(!getInput("orientation_tolerances", goal_.orientation_tolerances)){
      goal_.orientation_tolerances = orientation_tolerances_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: position_tolerances: "<<goal_.position_tolerances.at(1));

    if(!getInput("is_attached", goal_.is_attached)){
      goal_.is_attached = is_attached_;
    }
    ROS_DEBUG_STREAM_NAMED(action_name_+"_client", action_name_+"_client: is_attached: "<<goal_.is_attached);

}
  
BT::NodeStatus ComputePathActionClient::on_success()
{
  setOutput("plan", result_->plan);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  float time_for_wait = 5.0;
  BT::NodeBuilder builder =
    [time_for_wait](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::ComputePathActionClient>(
        name, "compute_path", config, time_for_wait, "");
    };

  factory.registerBuilder<man_behavior_tree_nodes::ComputePathActionClient>(
    "ComputePathArm", builder);
}