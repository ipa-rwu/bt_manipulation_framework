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
    io_msg_.request.fun = static_cast<int8_t>(IO_SERVICE_FUN_LEVEL_);

    if(goal->action.compare("open") == 0 || goal->action.compare("Open") == 0){
        io_msg_.request.pin = static_cast<int8_t>(1);
    }

    if(goal->action.compare("close") == 0 || goal->action.compare("Close") == 0){
        io_msg_.request.pin = static_cast<int8_t>(0);
    }

    io_msg_.request.state = 0.0;

    // ROS_INFO_STREAM_NAMED(getName(),getName() <<": goal: pin:" <<  goal->pin << " fun:"<<  goal->fun <<" action:" <<  goal->action );


    ROS_INFO_NAMED(getName(),"from req: pin: %d, fun: %d, action: %f \n", io_msg_.request.pin, io_msg_.request.fun, io_msg_.request.state );

    if(client_.call(io_msg_)){
        ROS_INFO_STREAM_NAMED(getName(),getName() << goal->action <<" gripper initialise : " << ((io_msg_.response.success==0)?"Failed":"Succeeded") );
        if(io_msg_.response.success==0){
            action_res_.success = 0;
            const std::string response = "FAILURE";
            as_->setAborted(action_res_, response);
        }
    }
    else{
        action_res_.success = 0;
        const std::string response = "FAILURE";
        ROS_INFO_STREAM_NAMED(getName(),getName() << ": 1st failed to call" );
    }

    io_msg_.request.state = 1.0;
    ros::Duration(0.5).sleep();
    if(client_.call(io_msg_)){
        ROS_INFO_STREAM_NAMED(getName(),getName() << goal->action <<" gripper do : " << ((io_msg_.response.success==0)?"Failed":"Succeeded") );
        if(io_msg_.response.success==0){
            action_res_.success = 0;
            const std::string response = "FAILURE";
            as_->setAborted(action_res_, response);
        }
    }
    else{
        action_res_.success = 0;
        const std::string response = "FAILURE";
        ROS_INFO_STREAM_NAMED(getName(),getName() << ": 2nd failed to call" );
    }

    io_msg_.request.state = 0.0;
    ros::Duration(0.5).sleep();
    if(client_.call(io_msg_)){
        ROS_INFO_STREAM_NAMED(getName(),getName() << goal->action <<" gripper finished : " << ((io_msg_.response.success==0)?"Failed":"Succeeded") );
        if(io_msg_.response.success==0){
            action_res_.success = 0;
            const std::string response = "FAILURE";
            as_->setAborted(action_res_, response);
        }
    }
    else{
        action_res_.success = 0;
        const std::string response = "FAILURE";
        ROS_INFO_STREAM_NAMED(getName(),getName() << ": 3rd failed to call" );
    }

    action_res_.success = 1;
    const std::string response = "SUCCESS";
    as_->setSucceeded(action_res_, response);

}

} //namespace
    
    // if(client_.call(io_msg_) && io_msg_.response.success){
    //     io_msg_.request.state = 1.0;
    //     if(client_.call(io_msg_) && io_msg_.response.success){
    //         io_msg_.request.state = 0;
    //         if(client_.call(io_msg_) && io_msg_.response.success){
    //             action_res_.success = 1;
    //             const std::string response = "SUCCESS";
    //             as_->setSucceeded(action_res_, response);
    //         }
    //         else{
    //             action_res_.success = 0;
    //             const std::string response = "FAILURE";
    //             ROS_INFO("1 failed");
    //             as_->setAborted(action_res_, response);
    //         }
    //     }
    //     else{
    //         action_res_.success = 0;
    //         const std::string response = "FAILURE";
    //         ROS_INFO("2 failed");
    //         as_->setAborted(action_res_, response);
    //     }
    // }
    // else{
    //     action_res_.success = 0;
    //     ROS_INFO("3 failed");
    //     const std::string response = "FAILURE";
    //     as_->setAborted(action_res_, response);
    // }



