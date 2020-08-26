#ifndef MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_
#define MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"
#include <actionlib/server/simple_action_server.h>
#include <man_msgs/ComputePathSkillAction.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <geometry_msgs/Pose.h>


namespace manipulator_skills
{

typedef actionlib::SimpleActionServer<man_msgs::ComputePathSkillAction> ComputePathActionServer;

class ArmComputePathSkill  : public ManipulatorSkill
{
public:
  ArmComputePathSkill(std::string group_name);
  ~ArmComputePathSkill();

  void initialize() override;

private:
  moveit_msgs::MoveItErrorCodes computePath(const man_msgs::ComputePathSkillGoalConstPtr& goal,
                                         moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                         moveit_msgs::MoveItErrorCodes error_code);

  void executeCB(const man_msgs::ComputePathSkillGoalConstPtr &goal);
  void AdjustTrajectoryToFixTimeSequencing(moveit_msgs::RobotTrajectory &trajectory);

  // void preemptComputerPathCallback();
  // void setComputerPathState(MoveGroupState state);

  // std::unique_ptr<actionlib::SimpleActionServer<man_msgs::ComputePathSkillAction> > compute_path_action_server_;

  ComputePathActionServer* as_;

  moveit::planning_interface::MoveGroupInterface *move_group_;

  std::string group_name_;
  std::string action_name_;
  ros::ServiceClient motion_plan_client_;
  
  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;

  bool isCartesianPath_;
  int replan_times_;
};
} //namespace manipulator_skills

#endif //MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_