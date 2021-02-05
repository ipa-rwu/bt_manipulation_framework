#include "manipulator_skills/skills/update_arm_goal.hpp"

#include <manipulator_skills/skill_names.hpp>

namespace manipulator_skills
{
ArmUpdateGoal::ArmUpdateGoal(
    std::string group_name,
    const ros::NodeHandle &private_node_handle) :
    ManipulatorSkill(UPDATE_ARM_GOAL),
    pnh_(private_node_handle),
    service_name_(UPDATE_ARM_GOAL),
    group_name_(group_name)
{
    this->initialize();
}

ArmUpdateGoal::~ArmUpdateGoal()
{
    // if(move_group_ != NULL)
    // delete move_group_;
}

void ArmUpdateGoal::initialize()
{
    // move_group_ = new moveit::planning_interface::MoveGroupInterface(group_name_);
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
    service_ = pnh_.advertiseService(UPDATE_ARM_GOAL, &ArmUpdateGoal::executeCB, this);

    tf_.reset(new tf2_ros::Buffer());
    tfListener_.reset(new tf2_ros::TransformListener(*tf_));

    // ROS_INFO_STREAM_NAMED(getName(), "start service" );
    // std::cout << move_group_->getCurrentPose().pose;
}

bool ArmUpdateGoal::executeCB(man_msgs::UpdateArmGoal::Request  &req,
                    man_msgs::UpdateArmGoal::Response &res)
{
    // current_state_ = move_group_->getCurrentState();
    // std::cout<< *current_state << std::endl;
    // moveit_msgs::RobotState robot_state;

    // robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
    // std::cout<< robot_state << std::endl;
    if (req.frame_id.compare(req.target.header.frame_id) != 0)
    {
        try
        {
            transformStamped_ = tf_->lookupTransform(req.frame_id, 
                                        req.target.header.frame_id,
                                        ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            return false;
        }



        tf2::Transform pose_to_tf;
        tf2::convert(transformStamped_.transform, pose_to_tf);
        tf2::Transform pose_old, pose_new;
        tf2::convert(req.target.pose, pose_old);
        pose_new = pose_old * pose_to_tf;
        
        // tf2::fromMsg(req.pose.pose, pose_to_tf);
        // tf2::Transform after_transfer_tf = pose_to_tf * transformStamped_;
        geometry_msgs::Transform current_msg = tf2::toMsg(pose_new);

        res.goal.header.frame_id = req.frame_id;
        res.goal.header.stamp = req.target.header.stamp;
        res.goal.pose.orientation.w = current_msg.rotation.w;
        res.goal.pose.orientation.x = current_msg.rotation.x;
        res.goal.pose.orientation.y = current_msg.rotation.y;
        res.goal.pose.orientation.z = current_msg.rotation.z;
        res.goal.pose.position.x = current_msg.translation.x;
        res.goal.pose.position.y = current_msg.translation.y;
        res.goal.pose.position.z = current_msg.translation.z;

    }
    else
    {
        res.goal.header.frame_id = req.frame_id;
        res.goal.header.stamp = req.target.header.stamp;
        res.goal.pose.orientation.w = req.target.pose.orientation.w;
        res.goal.pose.orientation.x = req.target.pose.orientation.x ;
        res.goal.pose.orientation.y = req.target.pose.orientation.y;
        res.goal.pose.orientation.z = req.target.pose.orientation.z;
        res.goal.pose.position.x = req.target.pose.position.x;
        res.goal.pose.position.y = req.target.pose.position.y;
        res.goal.pose.position.z = req.target.pose.position.z;
    }

    if (req.param != 0)
    {
        res.goal.pose.position.z += req.param;
    }

    // ROS_INFO_STREAM_NAMED("update_arm_goal_server", "[req] frame_id: "<< req.frame_id);

    // ROS_INFO_STREAM_NAMED("update_arm_goal_server", "[req] param: " << req.param);
    // ROS_INFO_STREAM_NAMED("update_arm_goal_server", "[req] target: "<< req.target);

    ROS_INFO_STREAM_NAMED("update_arm_goal_server", "[Goal for Arm]: "<< res.goal.pose);


    return true;
}





} // namespace







