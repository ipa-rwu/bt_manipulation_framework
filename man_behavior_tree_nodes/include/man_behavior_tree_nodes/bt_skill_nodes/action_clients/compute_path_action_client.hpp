#ifndef MAN_BEHAVIOR_TREE_NODES_COMPUTER_PATH_CLIENT_
#define MAN_BEHAVIOR_TREE_NODES_COMPUTER_PATH_CLIENT_

#include "man_behavior_tree_nodes/bt_action_client.hpp"
#include "man_msgs/ComputePathSkillAction.h"
#include <moveit_msgs/GetMotionPlan.h>

namespace man_behavior_tree_nodes
{
class EmptyClass{};
class ComputePathActionClient : public btActionClient<man_msgs::ComputePathSkillAction, 
                                                        man_msgs::ComputePathSkillGoal,
                                                        man_msgs::ComputePathSkillResultConstPtr,
                                                        man_msgs::ComputePathSkillFeedbackConstPtr,
                                                        EmptyClass>
{
public:
    ComputePathActionClient(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf,
        float time_for_wait,
        const std::string & subscribe_topic_name);
    
    void on_tick() override;

    BT::NodeStatus on_success() override;

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts(
        {
            BT::InputPort<geometry_msgs::PoseStamped>("goal", "Destination to plan to"),
            BT::InputPort<int>("replan_times", "times for replan"),
            // BT::InputPort<std::string>("end_effector", "robot end effector link"),
            BT::InputPort<std::string>("goal_name", ""),
            BT::InputPort<float>("allowed_planning_time", ""),
            BT::InputPort<float>("eef_step", ""),
            BT::InputPort<float>("jump_threshold", ""),
            BT::InputPort<float>("max_velocity_scaling_factor", ""),
            BT::InputPort<float>("max_acceleration_scaling_factor", ""),
            BT::InputPort<std::string>("planner_id", ""),
            BT::InputPort<std::string>("target_type", ""),
            BT::InputPort<std::vector<float>>("position_tolerances", ""),
            BT::InputPort<std::vector<float>>("orientation_tolerances", ""),
            BT::InputPort<bool>("is_attached", ""),
            
            BT::OutputPort<man_msgs::Plan>("plan", ""), 
        });
    }

private:
    bool first_time_{true};
    geometry_msgs::PoseStamped posestamp_;
    std::string active_target_;
    double goal_joint_tolerance_;
    double goal_position_tolerance_;
    double goal_orientation_tolerance_;

    // joint state goal
    robot_state::RobotStatePtr joint_state_target_;
    const robot_model::JointModelGroup* joint_model_group_;

    std::vector<float> position_tolerances_{0.1f, 0.1f, 0.1f};
    std::vector<float> orientation_tolerances_{0.1f, 0.1f, 0.1f};
    float max_velocity_scaling_factor_{1.0f};
    float max_acceleration_scaling_factor_{1.0f};
    float allowed_planning_time_{60.0f};
    std::string planner_id_{"RRTConnectkConfigDefault"};
    std::string target_type_;
    std::string named_goal_;
    int replan_times_{2};
    float eef_step_{0.01};
    float jump_threshold_{0.0};
    bool is_attached_{0};

};
} // namespace

#endif