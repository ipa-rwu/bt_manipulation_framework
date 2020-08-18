#ifndef MANIPULATOR_SKILLS_UPDATE_ARM_STATE_SKILL_
#define MANIPULATOR_SKILLS_UPDATE_ARM_STATE_SKILL_


#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"

#include <man_msgs/UpdateArmState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>



namespace manipulator_skills
{

class ArmUpdateState  : public ManipulatorSkill
{
public:
  ArmUpdateState(std::string group_name,
    const ros::NodeHandle &private_node_handle);
  ~ArmUpdateState();

  void initialize() override;

private:
  bool executeCB(man_msgs::UpdateArmState::Request  &req,
                    man_msgs::UpdateArmState::Response &res);

//   void computePathCallback(const man_msgs::ComputePathSkillGoalConstPtr& goal);


  ros::ServiceServer service_;

 moveit::planning_interface::MoveGroupInterface *move_group_;
//   std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::string group_name_;
  std::string service_name_;
  
  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;
  // shared_ptr    
  robot_state::RobotStatePtr current_state_;

};
} //namespace manipulator_skills

#endif //MANIPULATOR_SKILLS_UPDATE_ARM_STATE_SKILL_