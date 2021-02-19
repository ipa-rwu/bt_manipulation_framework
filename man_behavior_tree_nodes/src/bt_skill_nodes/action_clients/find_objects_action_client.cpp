#include "man_behavior_tree_nodes/bt_skill_nodes/action_clients/find_objects_action_client.hpp"

namespace man_behavior_tree_nodes
{

FindObjectsActionClient::FindObjectsActionClient(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf,
  float time_for_wait,
  const std::string & subscribe_topic_name)
: btActionClient<man_msgs::FindObjectsAction, 
                  man_msgs::FindObjectsGoal, 
                  man_msgs::FindObjectsResultConstPtr,
                  man_msgs::FindObjectsFeedbackConstPtr,
                  EmptyClass>
                  (xml_tag_name, action_name, conf, time_for_wait, subscribe_topic_name)
{
}


void FindObjectsActionClient::on_tick()
{
    getInput("container_A", goal_.container_a);
    getInput("container_B", goal_.container_b);
    getInput("marker_id", goal_.marker_id);
    getInput("frame_id", goal_.frame_id);
    // ROS_INFO_STREAM_NAMED("find_objects", "[goal] container_a.pose: "<< goal_.container_a);
    // ROS_INFO_STREAM_NAMED("find_objects", "[goal] container_b.pose: "<< goal_.container_b);
    // ROS_INFO_STREAM_NAMED("find_objects", "[goal] frame_id: "<< goal_.frame_id);
}


BT::NodeStatus FindObjectsActionClient::on_success()
{
  man_msgs::FindObjectsResultConstPtr result;
  setOutput("marker", result_->marker_pose);
  setOutput("container", result_->container_pose);
  // ROS_INFO_STREAM_NAMED("find_objects", "[result] marker: "<< result_->marker_pose);
  // ROS_INFO_STREAM_NAMED("find_objects", "[result] container: "<< result_->container_pose);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace man_behavior_tree_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  float time_for_wait = 20.0;
  BT::NodeBuilder builder =
    [&time_for_wait](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<man_behavior_tree_nodes::FindObjectsActionClient>(
        name, "find_objects", config, time_for_wait, "");
    };

  factory.registerBuilder<man_behavior_tree_nodes::FindObjectsActionClient>(
    "FindObjects", builder);
}