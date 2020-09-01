#include "manipulator_skills/skills/execute_gripper_trajectory.hpp"
#include <manipulator_skills/skill_names.hpp>

namespace manipulator_skills
{
GripperExecuteTrajectorySkill::GripperExecuteTrajectorySkill(std::string group_name) :
    ManipulatorSkill(EXECUTE_GRIPPER_TRAJECTORY_NAME),
    action_name_(EXECUTE_GRIPPER_TRAJECTORY_NAME),
    group_name_(group_name)
{
  this->initialize();
}

GripperExecuteTrajectorySkill::~GripperExecuteTrajectorySkill()
{
}

void GripperExecuteTrajectorySkill::initialize()
{
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
    // start the move action server
    as_.reset(new ExecuteGripperTrajectoryServer(root_node_handle_, EXECUTE_GRIPPER_TRAJECTORY_NAME, 
    boost::bind(&GripperExecuteTrajectorySkill::executeCB, this, _1), false));

    // robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    
    as_->start();
    ROS_INFO_STREAM_NAMED(getName(), "start action" );
}

void GripperExecuteTrajectorySkill::executeCB(const man_msgs::ExecuteGripperTrajectoryGoalConstPtr& goal)
{   
    std::string target = goal->action_name;
    move_group_->getCurrentState();
    move_group_->setNamedTarget(target);

    if (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS)
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



} // namespacev