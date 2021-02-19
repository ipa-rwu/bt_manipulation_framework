#ifndef MANIPULATOR_SKILLS_EXECUTE_TRAJECTORY_SKILL_
#define MANIPULATOR_SKILLS_EXECUTE_TRAJECTORY_SKILL_
#include <boost/bind.hpp>
#include <boost/thread.hpp> 
#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"
#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "man_msgs/ExecuteTrajectorySkillAction.h"

namespace manipulator_skills
{

typedef actionlib::SimpleActionServer<man_msgs:: ExecuteTrajectorySkillAction> ExecuteTrajectorySkillServer;

class ArmExecuteTrajectorySkill  : public ManipulatorSkill
{
public:
  ArmExecuteTrajectorySkill(std::string group_name,
    std::string service_name);
  ~ArmExecuteTrajectorySkill();

  void initialize() override;

private:
  void executeCB(const man_msgs::ExecuteTrajectorySkillGoalConstPtr &goal);
  void executePlan();
  void check();

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface::Plan> plan_;

  std::unique_ptr<ExecuteTrajectorySkillServer> as_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  // std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> > move_action_client_;
  std::string group_name_;
  std::string action_name_;

  man_msgs::ExecuteTrajectorySkillResult action_res_;
  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;

  std::unique_ptr<boost::thread> wait_execute_thread_;
  std::unique_ptr<boost::thread> check_thread_;

  moveit::planning_interface::MoveItErrorCode error_code_{moveit::planning_interface::MoveItErrorCode::TIMED_OUT};

  std::string error_convert(moveit::planning_interface::MoveItErrorCode error){
    if(error == moveit::planning_interface::MoveItErrorCode::FAILURE){
        return ("FAILURE");
    }
    if(error == moveit::planning_interface::MoveItErrorCode::PREEMPTED){
        return ("PREEMPTED");
    }
    if(error == moveit::planning_interface::MoveItErrorCode::SUCCESS){
        return ("SUCCEEDED");
    }
    if(error == moveit::planning_interface::MoveItErrorCode::TIMED_OUT){
        return ("TIMED_OUT");
    }
    if(error == moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED){
        return ("CONTROL_FAILED");
    } 
    if(error == moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN){
        return ("INVALID_MOTION_PLAN");
    }                
    else
        return ("Unknown");
  }

};
} //namespace manipulator_skills

#endif //MANIPULATOR_SKILLS_EXECUTE_TRAJECTORY_SKILL_