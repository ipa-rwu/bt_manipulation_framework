#include "manipulator_skills/skills/action_servers/execute_trajectory_skill.hpp"

namespace manipulator_skills
{
ArmExecuteTrajectorySkill::ArmExecuteTrajectorySkill(std::string group_name,
    std::string server_name) :
    ManipulatorSkill(server_name),
    action_name_(server_name),
    group_name_(group_name)
{
  this->initialize();
}

ArmExecuteTrajectorySkill::~ArmExecuteTrajectorySkill()
{
}

void ArmExecuteTrajectorySkill::initialize()
{
    // WebotsSkills webots_obj;
    // webotsRobotName_ = webots_obj.fixName();
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
    // start the move action server
    as_.reset(new ExecuteTrajectorySkillServer(root_node_handle_, action_name_, 
        boost::bind(&ArmExecuteTrajectorySkill::executeCB, this, _1), false)); 
    plan_.reset(new moveit::planning_interface::MoveGroupInterface::Plan);  
    as_->start();
    ROS_INFO_STREAM_NAMED(getName(),getName() << ": waitng for client" );
}

void ArmExecuteTrajectorySkill::executePlan(){
    try 
    { 
        ROS_WARN_STREAM_NAMED(action_name_, action_name_ + ": Start execution");
        error_code_ = move_group_->execute(*plan_);   
        ROS_WARN_STREAM_NAMED(action_name_, action_name_ + ": After error_code: " <<error_convert(error_code_));

    } 
    catch (boost::thread_interrupted&) 
    { 
       move_group_->stop();
    } 
}

void ArmExecuteTrajectorySkill::check(){
    try 
    { 
        while (!error_code_ == moveit::planning_interface::MoveItErrorCode::SUCCESS || 
            !error_code_ == moveit::planning_interface::MoveItErrorCode::FAILURE ||
            !error_code_ == moveit::planning_interface::MoveItErrorCode::PREEMPTED ||
             !error_code_ == moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED)
        {
            // check that preempt has not been requested by the client
            if (as_->isPreemptRequested() || !ros::ok())
            {
                // set the action state to preempted
                as_->setPreempted();
                wait_execute_thread_->interrupt();
            }

            if(error_code_ == moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED){
                break;
            }

            ROS_WARN_STREAM_NAMED(action_name_, action_name_ + ": in while loop error_code: " <<error_convert(error_code_));
        }
        ROS_WARN_STREAM_NAMED(action_name_, action_name_ + ":out while loop error_code: " <<error_convert(error_code_));

    } 
    catch (boost::thread_interrupted&) 
    { 
       move_group_->stop();
    } 
}

void ArmExecuteTrajectorySkill::executeCB(const man_msgs::ExecuteTrajectorySkillGoalConstPtr& goal)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    plan_->start_state_ = goal->plan.start_state;
    plan_->trajectory_ = goal->plan.trajectory;
    plan_->planning_time_ = goal->plan.planning_time;

    move_group_->setStartStateToCurrentState();

    wait_execute_thread_.reset(new boost::thread(boost::bind(&ArmExecuteTrajectorySkill::executePlan, this)));
    wait_execute_thread_->join();

    check_thread_.reset(new boost::thread(boost::bind(&ArmExecuteTrajectorySkill::check, this)));
    check_thread_->join();

    ROS_WARN_STREAM_NAMED(action_name_, action_name_ + ": error_code: " <<error_convert(error_code_));
    
    if (error_code_ == moveit::planning_interface::MoveItErrorCode::SUCCESS) 
    {
        // ROS_INFO("Moving to home pose SUCCESSFUL");
        action_res_.success = 1;
        const std::string response = "SUCCESS";
        as_->setSucceeded(action_res_, response);
    } 
    if (error_code_ == moveit::planning_interface::MoveItErrorCode::FAILURE)
    {
        action_res_.success = 0;
        const std::string response = "FAILURE";
        as_->setAborted(action_res_, response);
    }

    if (error_code_ == moveit::planning_interface::MoveItErrorCode::PREEMPTED)
    {
        action_res_.success = 0;
        const std::string response = "PREEMPTED";
        as_->setPreempted(action_res_, response);
    }

    if (error_code_ == moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED)
    {
        action_res_.success = 0;
        const std::string response = "CONTROL_FAILED";
        as_->setPreempted(action_res_, response);
    }
}

} // namespace