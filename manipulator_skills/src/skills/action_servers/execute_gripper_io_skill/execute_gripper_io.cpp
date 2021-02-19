#include "manipulator_skills/skills/action_servers/execute_gripper_io.hpp"
#include <iostream>

namespace manipulator_skills
{

ExecuteGripperIOSkill::ExecuteGripperIOSkill(std::string group_name,
    std::string server_name) :
    ManipulatorSkill(server_name),
    action_name_(server_name),
    group_name_(group_name)
{
  this->initialize();
}

ExecuteGripperIOSkill::~ExecuteGripperIOSkill()
{
}

void ExecuteGripperIOSkill::initialize()
{
    // start the move action server
    as_.reset(new ExecuteGripperIOServer(root_node_handle_, action_name_, 
        boost::bind(&ExecuteGripperIOSkill::executeCB, this, _1), false)); 
    as_->start();
    client_ =  pnh_.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
    ROS_INFO_STREAM_NAMED(getName(),getName() << ": waitng for call" );
}

void ExecuteGripperIOSkill::executeCB(const man_msgs::ExecuteGripperIOGoalConstPtr &goal)
{
    ur_msgs::SetIO io_msg_;
    io_msg_.request.fun = (int8_t)goal->fun;

    if(goal->action.compare("open") == 0 || goal->action.compare("Open") == 0){
        io_msg_.request.pin = (int8_t)1;
    }

    if(goal->action.compare("close") == 0 || goal->action.compare("Close") == 0){
        io_msg_.request.pin = (int8_t)0;
    }

    io_msg_.request.state = 0.0;

    ROS_DEBUG_NAMED(getName(),"pin: %d, fun: %d, action: %f \n", io_msg_.request.pin, io_msg_.request.fun, io_msg_.request.state );

    ROS_INFO_STREAM_NAMED(getName(),getName() <<": pin:" <<  goal->fun << " fun:"<<  goal->pin <<" action:" <<  goal->action );

    if(client_.call(io_msg_) && io_msg_.response.success){
        io_msg_.request.state = 1.0;
        if(client_.call(io_msg_) && io_msg_.response.success){
            io_msg_.request.state = 0;
            if(client_.call(io_msg_) && io_msg_.response.success){
                action_res_.success = 1;
                const std::string response = "SUCCESS";
                as_->setAborted(action_res_, response);
            }
            else{
                action_res_.success = 0;
                const std::string response = "FAILURE";
                as_->setAborted(action_res_, response);
            }
        }
        else{
            action_res_.success = 0;
            const std::string response = "FAILURE";
            as_->setAborted(action_res_, response);
        }
    }
    else{
        action_res_.success = 0;
        const std::string response = "FAILURE";
        as_->setAborted(action_res_, response);
    }

}

} //namespace