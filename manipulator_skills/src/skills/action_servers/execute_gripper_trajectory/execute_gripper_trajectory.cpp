#include "manipulator_skills/skills/action_servers/execute_gripper_trajectory.hpp"

namespace manipulator_skills
{
GripperExecuteTrajectorySkill::GripperExecuteTrajectorySkill(std::string group_name,
    std::string server_name) :
    ManipulatorSkill(server_name),
    action_name_(server_name),
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
    as_.reset(new ExecuteGripperTrajectoryServer(root_node_handle_, action_name_, 
    boost::bind(&GripperExecuteTrajectorySkill::executeCB, this, _1), false));    
    as_->start();
    ROS_INFO_STREAM_NAMED(getName(),getName() << ": waitng for client" );
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