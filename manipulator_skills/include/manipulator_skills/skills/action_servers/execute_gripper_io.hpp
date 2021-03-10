#ifndef MANIPULATOR_SKILLS_EXECUTE_GRIPPER_IO_SKILL_
#define MANIPULATOR_SKILLS_EXECUTE_GRIPPER_IO_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"
#include "ur_msgs/SetIO.h"
#include "man_msgs/ExecuteGripperIOAction.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>  
#include <actionlib/server/simple_action_server.h>

namespace manipulator_skills
{
typedef actionlib::SimpleActionServer<man_msgs:: ExecuteGripperIOAction> ExecuteGripperIOServer;
class ExecuteGripperIOSkill  : public ManipulatorSkill
{
public:
  ExecuteGripperIOSkill(std::string group_name,
    std::string service_name);
  ~ExecuteGripperIOSkill();

  void initialize() override;

private:
  void executeCB(const man_msgs::ExecuteGripperIOGoalConstPtr &goal);
  
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    // shared_ptr    
    ros::ServiceClient client_;

    std::unique_ptr<ExecuteGripperIOServer> as_;
    man_msgs::ExecuteGripperIOResult action_res_;

    std::string group_name_;
    std::string action_name_;

    const int IO_SERVICE_FUN_LEVEL_ = 1; 

}; //namespace manipulator_skills
}

#endif //MANIPULATOR_SKILLS_EXECUTE_GRIPPER_IO_SKILL_