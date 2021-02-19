#include "man_behavior_tree_nodes/bt_skill_nodes/execute_trajectory_arm_action_client.hpp"

namespace man_behavior_tree_nodes
{

ExecuteTrajectoryActionClient::ExecuteTrajectoryActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait,
  const std::string & subscribe_topic_name)
: btActionClient<man_msgs::ExecuteTrajectorySkillAction, 
                man_msgs::ExecuteTrajectorySkillGoal,
                man_msgs::ExecuteTrajectorySkillResultConstPtr,
                man_msgs::ExecuteTrajectorySkillFeedbackConstPtr,
                webots_ros::BoolStamped::ConstPtr>
                  (xml_tag_name, action_name, conf, time_for_wait, subscribe_topic_name)
{
}

void ExecuteTrajectoryActionClient::on_tick()
{
  getInput("plan", goal_.plan);

}

void ExecuteTrajectoryActionClient::on_wait_for_result()
{
  if(!finished_){
    if(touch_data_){
      mtx_interrupt_.lock();
      interrupt_ = true;
      mtx_interrupt_.unlock();
    }
  }
}

void ExecuteTrajectoryActionClient::subCallback(webots_ros::BoolStamped::ConstPtr msg)
{
  mtx_sub_.lock();
  touch_data_ = msg->data;
  mtx_sub_.unlock();

}

// void ExecuteTrajectoryActionClient::subCallback(webots_ros::BoolStamped::ConstPtr msg)
// {
//     touch_data_ = msg->data;
//     if (touch_data_ && finished_ == false)
//     {
//         // ROS_INFO("[touch sensor call back]: touched");
//         collision_happened_ = true;
//     }
//     touch_data_ = false;
// }
  
BT::NodeStatus ExecuteTrajectoryActionClient::on_success()
{
    // std::cout << "ExecuteTrajectoryActionClient on_success" << std::endl;

    success_ = result_->success;
    // for debug
    setOutput("result", success_);

    if(success_ == 1)
        return BT::NodeStatus::SUCCESS;

    if(success_ == 0)
        return BT::NodeStatus::FAILURE;
}

}  // namespace nav2_behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  float time_for_wait = 20.0;
  std::string subscribe_topic_name = "/container_A/touch_sensor";
  BT::NodeBuilder builder =
    [&time_for_wait, &subscribe_topic_name](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::ExecuteTrajectoryActionClient>(
        name, "execute_trajectory_arm", config, time_for_wait, "/container_A/touch_sensor");
    };

  factory.registerBuilder<man_behavior_tree_nodes::ExecuteTrajectoryActionClient>(
    "ExecuteTrajectoryArm", builder);
}