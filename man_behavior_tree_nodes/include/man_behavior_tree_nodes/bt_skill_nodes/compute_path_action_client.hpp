#ifndef MAN_BEHAVIOR_TREE_NODES_COMPUTER_PATH_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_COMPUTER_PATH_CLIENT_


#include "man_behavior_tree_nodes/bt_action_client.h"
#include "man_msgs/ComputePathSkillAction.h"
#include <moveit_msgs/GetMotionPlan.h>

namespace man_behavior_tree_nodes
{
class ComputePathActionClient : public btActionClient<moveit_msgs::MoveGroupAction, moveit_msgs::MoveGroupGoal>
{
    public:
    ComputePathActionClient(
        const std::string & xml_tag_name,
        const std::string & action_name,
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