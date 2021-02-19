#ifndef MANIPULATOR_SKILLS_EXECUTE_TRAJECTORY_GRIPPER_SKILL_
#define MANIPULATOR_SKILLS_EXECUTE_TRAJECTORY_GRIPPER_SKILL_
#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"
#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "man_msgs/ExecuteGripperTrajectoryAction.h"

namespace manipulator_skills
{

typedef actionlib::SimpleActionServer<man_msgs::ExecuteGripperTrajectoryAction> ExecuteGripperTrajectoryServer;

class GripperExecuteTrajectorySkill  : public ManipulatorSkill
{
public:
  GripperExecuteTrajectorySkill(std::string group_name,
    std::string service_name);
  ~GripperExecuteTrajectorySkill();

  void initialize() override;

private:
  void executeCB(const man_msgs::ExecuteGripperTrajectoryGoalConstPtr &goal);
  void executePlan();

  moveit::planning_interface::MoveItErrorCode error_code_{moveit::planning_interface::MoveItErrorCode::TIMED_OUT};

  std::unique_ptr<ExecuteGripperTrajectoryServer> as_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::string group_name_;
  std::string action_name_;

  man_msgs::ExecuteGripperTrajectoryResult action_res_;

  // public ros node handle
  ros::NodeHandle nh_;
  // private ros node handle
  ros::NodeHandle pnh_;
};
} //namespace manipulator_skills

#endif //MANIPULATOR_SKILLS_EXECUTE_TRAJECTORY_GRIPPER_SKILL_