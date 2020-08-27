#ifndef MANIPULATOR_SKILLS_UPDATE_ARM_GOAL_SKILL_
#define MANIPULATOR_SKILLS_UPDATE_ARM_GOAL_SKILL_

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"

#include <man_msgs/UpdateArmGoal.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include "tf2/convert.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace manipulator_skills
{

class ArmUpdateGoal  : public ManipulatorSkill
{
public:
  ArmUpdateGoal(std::string group_name,
    const ros::NodeHandle &private_node_handle);
  ~ArmUpdateGoal();

  void initialize() override;

private:
    bool executeCB(man_msgs::UpdateArmGoal::Request  &req,
                        man_msgs::UpdateArmGoal::Response &res);

    bool coordination_transform(std::string target_frame, std::string source_frame);


    //   void computePathCallback(const man_msgs::ComputePathSkillGoalConstPtr& goal);


    ros::ServiceServer service_;

    // moveit::planning_interface::MoveGroupInterface *move_group_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    std::string group_name_;
    std::string service_name_;
  
    // public ros node handle
    ros::NodeHandle nh_;
    // private ros node handle
    ros::NodeHandle pnh_;
    // shared_ptr    
    robot_state::RobotStatePtr current_state_;

    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    std::shared_ptr<tf2_ros::Buffer> tf_;

    geometry_msgs::TransformStamped transformStamped_;


};
} //namespace manipulator_skills

#endif