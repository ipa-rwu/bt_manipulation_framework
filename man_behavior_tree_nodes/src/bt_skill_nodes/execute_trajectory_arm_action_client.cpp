#include "man_behavior_tree_nodes/bt_skill_nodes/execute_trajectory_arm_action_client.hpp"

namespace man_behavior_tree_nodes
{

ExecuteTrajectoryActionClient::ExecuteTrajectoryActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait,
  const std::string & subscribe_topic_name_)
: btActionClient<man_msgs::ExecuteTrajectorySkillAction, 
                man_msgs::ExecuteTrajectorySkillGoal,
                man_msgs::ExecuteTrajectorySkillResultConstPtr,
                webots_ros::BoolStamped::ConstPtr>
                  (xml_tag_name, action_name, conf, time_for_wait, subscribe_topic_name_)
{
}

// void ExecuteTrajectoryActionClient::initialize()
// {
//     WebotsSkills webots_obj;
//     webotsRobotName_ = webots_obj.fixName();
//     ROS_INFO_STREAM_NAMED("ExecuteTrajectoryActionClient", "webots robot name: " << webotsRobotName_ );

//     touch_sensor_topic_name_ = "/container_A" + webotsRobotName_ + "/touch_sensor/value";
//     touch_sensor_sub_ = pnh_.subscribe(touch_sensor_topic_name_,
//                           1,
//                           &ExecuteTrajectoryActionClient::TouchsensorCallback,
//                           this);
// }

// void ExecuteTrajectoryActionClient::TouchsensorCallback(const webots_ros::BoolStamped::ConstPtr& touchsensor_msg)
// {
//   // msg: {"data": "start"}
//   result_touchsensor_ = touchsensor_msg->data;
// //   std::cout << result_touchsensor <<std::endl;

// }

void ExecuteTrajectoryActionClient::on_tick()
{
    // PoseStamped "goal"
    // moveit_msgs::MotionPlanRequest& request;

    getInput("plan", goal_.plan);

}

void ExecuteTrajectoryActionClient::on_wait_for_result()
{
  std::cout << "ExecuteTrajectoryActionClient on_wait_for_result" << std::endl;
  if (touch_data_ == true)
  {
    collision_happened_  = true;
  }

}
  
BT::NodeStatus ExecuteTrajectoryActionClient::on_success()
{
    std::cout << "ExecuteTrajectoryActionClient on_success" << std::endl;

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