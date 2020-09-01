#include "manipulator_skills/skills/execute_trajectory_skill.hpp"
#include <manipulator_skills/skill_names.hpp>

namespace manipulator_skills
{
ArmExecuteTrajectorySkill::ArmExecuteTrajectorySkill(std::string group_name) :
    ManipulatorSkill(EXECUTE_TRAJECTORY_NAME),
    action_name_(EXECUTE_TRAJECTORY_NAME),
    group_name_(group_name)
{
  this->initialize();
}

ArmExecuteTrajectorySkill::~ArmExecuteTrajectorySkill()
{
}

void ArmExecuteTrajectorySkill::initialize()
{
    // move_group_ = new moveit::planning_interface::MoveGroupInterface(group_name_);
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
    // start the move action server
    as_.reset(new ExecuteTrajectorySkillServer(root_node_handle_, EXECUTE_TRAJECTORY_NAME, 
    boost::bind(&ArmExecuteTrajectorySkill::executeCB, this, _1), false));

    // robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    
    as_->start();
    ROS_INFO_STREAM_NAMED(getName(), "start action" );
}

void ArmExecuteTrajectorySkill::executeCB(const man_msgs::ExecuteTrajectorySkillGoalConstPtr& goal)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    plan.start_state_ = goal->plan.start_state;
    plan.trajectory_ = goal->plan.trajectory;
    plan.planning_time_ = goal->plan.planning_time;


    move_group_->setStartStateToCurrentState();
    if (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        action_res_.success = 1;
        const std::string response = "SUCCESS";
        as_->setSucceeded(action_res_, response);
    }
    else
    {
        action_res_.success = 0;
        const std::string response = "FAILURE";
        as_->setAborted(action_res_, response);
    }

}



} // namespace