#ifndef MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_
#define MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_

#include <memory>

#include "ros/ros.h"
#include "manipulator_skills/skill.hpp"
#include <actionlib/server/simple_action_server.h>
#include <man_msgs/ComputePathSkillAction.h>

#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <moveit_msgs/PlanningScene.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>

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
  moveit_msgs::MoveItErrorCodes createPlanWithPlanner(const man_msgs::ComputePathSkillGoalConstPtr& goal,
                                         moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                         moveit_msgs::MoveItErrorCodes error_code);

  void executeCB(const man_msgs::ComputePathSkillGoalConstPtr &goal);
  void AdjustTrajectoryToFixTimeSequencing(moveit_msgs::RobotTrajectory &trajectory);

  moveit_msgs::Constraints updateGoal(std::vector<double> position_tolerances, 
                    std::vector<double> orientation_tolerances, 
                    const geometry_msgs::PoseStamped &pose_target,
                    std::string& end_effector);

  moveit_msgs::MoveItErrorCodes createPlanWithPipeline(moveit_msgs::Constraints& pose_constraints,
                                                        std::string &group_name,
                                                        std::string& planner_id,
                                                        float max_velocity_scaling_factor,
                                                        int32_t replan_times,
                                                        float allowed_planning_time,
                                                        const moveit_msgs::RobotState &start_robot_state,
                                                        moveit::planning_interface::MoveGroupInterface::Plan& plan);

  moveit_msgs::MoveItErrorCodes createPlanInterface(const geometry_msgs::PoseStamped& pose_target, 
                                                    float eef_step,
                                                    float jump_threshold,
                                                    int32_t replan_times,
                                                    const moveit_msgs::RobotState &start_robot_state,
                                                    moveit::planning_interface::MoveGroupInterface::Plan& plan);
  
  moveit_msgs::MoveItErrorCodes createPlanInterface(std::string &pose_name, 
                                                    std::string& planner_id,
                                                    float max_velocity_scaling_factor,
                                                    int32_t replan_times,
                                                    float allowed_planning_time,
                                                    const moveit_msgs::RobotState &start_robot_state,
                                                    moveit::planning_interface::MoveGroupInterface::Plan& plan);
                                                  


  // void preemptComputerPathCallback();
  // void setComputerPathState(MoveGroupState state);

  // std::unique_ptr<actionlib::SimpleActionServer<man_msgs::ComputePathSkillAction> > compute_path_action_server_;
  std::unique_ptr<ComputePathActionServer> as_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::unique_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :planning_scene:`PlanningScene` that maintains the state of
  // the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene_;  // We can now setup the PlanningPipeline object, which will use the ROS parameter server
  // to determine the set of request adapters and the planning plugin to use
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  man_msgs::ComputePathSkillResult action_res_;

  std::string group_name_;
  std::string action_name_;

    // using motion planning pipeline

  ros::ServiceClient motion_plan_client_;
  
  // public ros node handle
  ros::NodeHandle nh_{""};
  // private ros node handle
  ros::NodeHandle pnh_{"~"};

  int replan_times_{2};

  moveit::core::RobotStatePtr current_state_;
  moveit_msgs::RobotState robot_state_;
  moveit_msgs::Constraints pose_constraints_;

  std::string end_effector_;
  std::vector<double> position_tolerances_{(3,0.15f)};
  std::vector<double> orientation_tolerances_{(3,0.15f)};
  float max_velocity_scaling_factor_;
  float max_acceleration_scaling_factor_;
  float allowed_planning_time_{60.0f};
  geometry_msgs::PoseStamped pose_goal_;

  std::string planner_id_{"RRTConnectkConfigDefault"};

};
} //namespace manipulator_skills

#endif //MANIPULATOR_SKILLS_COMPUTER_PATH_SKILL_