#include "manipulator_skills/skills/update_arm_state.hpp"

#include <manipulator_skills/skill_names.hpp>


namespace manipulator_skills
{
ArmUpdateState::ArmUpdateState(
    std::string group_name,
    const ros::NodeHandle &private_node_handle) :
    ManipulatorSkill(UPDATE_ARM_STATE),
    pnh_(private_node_handle),
    service_name_(UPDATE_ARM_STATE),
    group_name_(group_name)
{
    this->initialize();
}

ArmUpdateState::~ArmUpdateState()
{
    // if(move_group_ != NULL)
    // delete move_group_;
}

void ArmUpdateState::initialize()
{
    move_group_ = new moveit::planning_interface::MoveGroupInterface(group_name_);

    service_ = pnh_.advertiseService(UPDATE_ARM_STATE, &ArmUpdateState::executeCB, this);
    ROS_INFO_STREAM_NAMED(getName(), "start service" );
    // std::cout << move_group_->getCurrentPose().pose;
}

bool ArmUpdateState::executeCB(man_msgs::UpdateArmState::Request  &req,
                    man_msgs::UpdateArmState::Response &res)
{
    ROS_INFO("get request");
    // current_state_ = move_group_->getCurrentState();
    robot_state::RobotStatePtr current_state =  move_group_->getCurrentState();
    // std::cout<< *current_state << std::endl;
    // moveit_msgs::RobotState robot_state;

    // robot_state::robotStateToRobotStateMsg(*current_state,robot_state);
    // std::cout<< robot_state << std::endl;
    if(req.isattached)
    {

    }
    else
    {

    }
    robot_state::robotStateToRobotStateMsg(*current_state,res.arm_state);
    return true;
}




} // namespace







