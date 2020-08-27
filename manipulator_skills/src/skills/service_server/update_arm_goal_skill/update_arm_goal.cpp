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

    ROS_INFO_STREAM_NAMED(getName(), "start service" );
    // std::cout << move_group_->getCurrentPose().pose;
}

bool ArmUpdateGoal::executeCB(man_msgs::UpdateArmGoal::Request  &req,
                    man_msgs::UpdateArmGoal::Response &res)
{
    ROS_INFO("get request");
    // current_state_ = move_group_->getCurrentState();
    // std::cout<< *current_state << std::endl;
    // moveit_msgs::RobotState robot_state;

    // robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
    // std::cout<< robot_state << std::endl;
    if (req.frame_id.compare(req.pose.header.frame_id) != 0)
    {
        if (coordination_transform(req.frame_id, req.pose.header.frame_id))
        {

            tf2::Transform pose_to_tf;
            tf2::convert(transformStamped_.transform, pose_to_tf);
            tf2::Transform pose_old, pose_new;
            tf2::convert(req.pose.pose, pose_old);
            pose_new = pose_old * pose_to_tf;
            
            // tf2::fromMsg(req.pose.pose, pose_to_tf);
            // tf2::Transform after_transfer_tf = pose_to_tf * transformStamped_;
            geometry_msgs::Transform current_msg = tf2::toMsg(pose_new);

            res.pose.header.frame_id = req.pose.header.frame_id;
            res.pose.header.stamp = req.pose.header.stamp;
            res.pose.pose.orientation.w = current_msg.rotation.w;
            res.pose.pose.orientation.x = current_msg.rotation.x;
            res.pose.pose.orientation.y = current_msg.rotation.y;
            res.pose.pose.orientation.z = current_msg.rotation.z;
            res.pose.pose.position.x = current_msg.translation.x;
            res.pose.pose.position.y = current_msg.translation.y;
            res.pose.pose.position.z = current_msg.translation.z;
        }
    }
    return true;
}


bool ArmUpdateGoal::coordination_transform(std::string target_frame, std::string source_frame)
{
    try{
    transformStamped_ = tf_->lookupTransform(target_frame, 
                                source_frame,
                                ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    return false;
    }
    return true;
}


} // namespace







