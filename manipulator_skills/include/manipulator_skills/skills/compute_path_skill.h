#ifndef MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_
#define MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.h"
#include <actionlib/server/simple_action_server.h>
#include <man_msgs/ComputePathSkillAction.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>

namespace manipulator_skills
{

typedef actionlib::SimpleActionServer<man_msgs::ComputePathSkillAction> ComputePathActionServer;

class ArmComputePathSkill  : public ManipulatorSkill
{
public:
  ArmComputePathSkill(std::string);
  ~ArmComputePathSkill();

  void initialize() override;

private:
  void computePathCallback(const man_msgs::ComputePathSkillGoalConstPtr& goal);
  bool computePath(const man_msgs::ComputePathSkillGoalConstPtr& goal,
                   man_msgs::ComputePathSkillResult& action_res,
                   moveit_msgs::MotionPlanResponse &res);
  // void preemptComputerPathCallback();
  // void setComputerPathState(MoveGroupState state);

  // std::unique_ptr<actionlib::SimpleActionServer<man_msgs::ComputePathSkillAction> > compute_path_action_server_;

  ComputePathActionServer* as_;

  std::string action_name_;
  ros::ServiceClient motion_plan_client_;
  
  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;

  void executeCB(const man_msgs::ComputePathSkillGoalConstPtr &goal);
};
} //namespace manipulator_skills

#endif //MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_