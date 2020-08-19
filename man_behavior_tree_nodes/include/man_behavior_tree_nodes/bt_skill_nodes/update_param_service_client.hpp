#ifndef MAN_BEHAVIOR_TREE_NODES_UPDATE_PARAM_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_UPDATE_PARAM_CLIENT_

#include "man_msgs/UpdateParam.h"
#include "man_behavior_tree_nodes/bt_service_client.hpp"

namespace man_behavior_tree_nodes
{
class UpdateParamclient : public btServiceClient<man_msgs::UpdateParam>
{
    public:
    ComputePathActionClient(
        const std::string & xml_tag_name,
        const std::string & service_name,
        const BT::NodeConfiguration & conf);
    
    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::OutputPort<moveit::planning_interface::MoveGroupInterface::Plan>("plan", "Plan created by ComputePathActionClient node"),
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Destination to plan to"),
            BT::InputPort<robot_state::RobotStatePtr>("arm_state", "arm state"),
            BT::InputPort<int>("replan_times", "times for replan"),
            BT::InputPort<std::string>("end_effector", "robot end effector link"),
            BT::InputPort<std::string>("group_name", ""),
            BT::InputPort<std::string>("planner_id", ""),
        });
    }

private:
  bool first_time_{true};
  geometry_msgs::msg::PoseStamped posestamp_;
  std::string active_target_;
  double goal_joint_tolerance_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;

  // joint state goal
  robot_state::RobotStatePtr joint_state_target_;
  const robot_model::JointModelGroup* joint_model_group_;
};
} // namespace

#endif