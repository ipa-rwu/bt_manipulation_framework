#include "manipulator_skills/skills/service_servers/update_arm_goal.hpp"

namespace manipulator_skills
{
ArmUpdateGoal::ArmUpdateGoal(
    std::string service_name,
    std::string group_name,
    const ros::NodeHandle &private_node_handle) :
    ManipulatorSkill(service_name),
    pnh_(private_node_handle),
    service_name_(service_name),
    group_name_(group_name)
{
    this->initialize();
}

ArmUpdateGoal::~ArmUpdateGoal()
{
}

void ArmUpdateGoal::initialize()
{
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
    service_ = pnh_.advertiseService(service_name_, &ArmUpdateGoal::executeCB, this);

    tf_.reset(new tf2_ros::Buffer());
    tfListener_.reset(new tf2_ros::TransformListener(*tf_));
    ROS_INFO_STREAM_NAMED(getName(), getName() << ": waitng for call" );
}

bool ArmUpdateGoal::executeCB(man_msgs::UpdateArmGoal::Request  &req,
                    man_msgs::UpdateArmGoal::Response &res)
{
    if (req.frame_id.compare(req.target.header.frame_id) != 0)
    {
        ROS_INFO_STREAM_NAMED(getName(), getName()<< ":  req.frame_id: " << req.frame_id << ":  req.target.header.frame_id: " << req.target.header.frame_id);
        try
        {
            transformStamped_ = tf_->lookupTransform(req.frame_id, 
                                        req.target.header.frame_id,
                                        ros::Time(0));
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_ERROR_STREAM_NAMED(getName(), getName()<< ":  tf: " << ex.what());
            return false;
        }

        tf2::Transform pose_to_tf;
        tf2::convert(transformStamped_.transform, pose_to_tf);
        tf2::Transform pose_old, pose_new;
        tf2::convert(req.target.pose, pose_old);
        pose_new = pose_old * pose_to_tf;
        
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
        ROS_INFO_STREAM_NAMED(getName(), getName() << ": did tf transfer");

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
    ROS_INFO_STREAM_NAMED(getName(), getName() <<": arm goal:\n" << res.goal.header.frame_id << "\n" <<res.goal.pose);
    return true;
}
} // namespace