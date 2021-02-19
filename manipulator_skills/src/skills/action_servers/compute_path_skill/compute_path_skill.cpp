
#include "manipulator_skills/skills/action_servers/compute_path_skill.hpp"
#include <manipulator_skills/skill_names.hpp>

namespace manipulator_skills
{
ArmComputePathSkill::ArmComputePathSkill(std::string group_name,
    std::string server_name) :
    ManipulatorSkill(server_name),
    action_name_(server_name),
    group_name_(group_name)
{
  this->initialize();
}

ArmComputePathSkill::~ArmComputePathSkill()
{
}

void ArmComputePathSkill::initialize()
{
  move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));

  robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>("robot_description");

  robot_model_ = robot_model_loader_->getModel();

  motion_plan_client_ = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");

  // start the move action server
  as_.reset(new ComputePathActionServer(root_node_handle_, action_name_, 
    boost::bind(&ArmComputePathSkill::executeCB, this, _1), false));

  // robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
  
  as_->start();
  ROS_INFO_STREAM_NAMED(getName(),getName() << ": waitng for client" );
}

void ArmComputePathSkill::executeCB(const man_msgs::ComputePathSkillGoalConstPtr& goal)
{
  // initial req
  end_effector_ = goal->end_effector;
  group_name_ = goal->group_name;
  planner_id_ = goal->planner_id;
  replan_times_ = goal->num_planning_attempts;
  pose_goal_ = goal->goal;
  if((float)goal->max_acceleration_scaling_factor != 0.0)
    max_acceleration_scaling_factor_ = (float) goal->max_acceleration_scaling_factor;
  if((float)goal->max_velocity_scaling_factor != 0.0)
    max_velocity_scaling_factor_ = (float) goal->max_velocity_scaling_factor;

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::MoveItErrorCodes error_code;
    // compute path with pose
  if(goal->target_type.compare("Pose") == 0)
  {   
    if(goal->goal.header.frame_id.empty()){
      ROS_ERROR_STREAM_NAMED(getName(),getName() << ": this goal does not provide frameID");
      error_code.val == moveit::planning_interface::MoveItErrorCode::FAILURE;
    }
    else{
      pose_constraints_ = updateGoal(position_tolerances_, orientation_tolerances_, goal->goal, end_effector_);
      error_code = createPlanInterface(pose_constraints_, 
                                          group_name_, 
                                          planner_id_,
                                          max_velocity_scaling_factor_,
                                          max_acceleration_scaling_factor_,
                                          replan_times_, 
                                          goal->allowed_planning_time,
                                          plan);
    }
  }

  // compute path with pose name
  if(goal->target_type.compare("Name") == 0)
  {
    std::string named_goal = goal->named_goal;
    if(named_goal.empty()){
      ROS_ERROR_STREAM_NAMED(getName(),getName() << ": this goal does not give goal name");
      error_code.val == moveit::planning_interface::MoveItErrorCode::FAILURE;
    }
    else{
      error_code = createPlanInterface(named_goal, 
                                    planner_id_,
                                    max_velocity_scaling_factor_,
                                    max_acceleration_scaling_factor_,
                                    goal->num_planning_attempts, 
                                    goal->allowed_planning_time,
                                    plan);
    }
  }

  // compute path with Cartesian Paths
  if(goal->target_type.compare("Cartesian") == 0)
  {
    if(goal->jump_threshold != -1){
      jump_threshold_ = goal->jump_threshold;
    }
    if(goal->eef_step != -1){
      eef_step_ = goal->eef_step;
    }

    error_code = createPlanInterface(goal->goal, 
                          eef_step_, 
                          jump_threshold_,
                          goal->num_planning_attempts, 
                          plan);
  }

    if (error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      action_res_.plan.start_state = plan.start_state_;
      action_res_.plan.trajectory = plan.trajectory_;
      action_res_.plan.planning_time = plan.planning_time_;

      const std::string response = "SUCCESS";
      as_->setSucceeded(action_res_, response);
    }
    else
    {
      const std::string response = "FAILURE";
      as_->setAborted(action_res_, response);
    }
}

moveit_msgs::Constraints ArmComputePathSkill::updateGoal(std::vector<double> position_tolerances, 
                                      std::vector<double> orientation_tolerances, 
                                      const geometry_msgs::PoseStamped &pose_target,
                                      std::string& end_effector)
{
  moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
                                      end_effector, pose_target, position_tolerances, orientation_tolerances);
  return pose_goal;
}

// compute path for pose using motion planning pipeline
moveit_msgs::MoveItErrorCodes ArmComputePathSkill::createPlanInterface(moveit_msgs::Constraints& pose_constraints,
                                                  std::string& group_name,
                                                  std::string& planner_id,
                                                  float max_velocity_scaling_factor,
                                                  float max_acceleration_scaling_factor,
                                                  int32_t replan_times,
                                                  float allowed_planning_time,
                                                  moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    moveit_msgs::MoveItErrorCodes error_code;
    error_code.val = moveit::planning_interface::MoveItErrorCode::FAILURE;

    // creating motion plan request
    moveit_msgs::GetMotionPlan motion_plan;
    moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
    moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;

    req.goal_constraints.clear();
  
    mtx_current_state_.lock();
    auto current_state = move_group_->getCurrentState();
    robot_state::robotStateToRobotStateMsg(*current_state ,robot_state_);
    mtx_current_state_.unlock();

    req.start_state = robot_state_;
    // req.start_state.is_diff = true;
    req.group_name = group_name;
    req.planner_id = planner_id;
    req.goal_constraints.push_back(pose_constraints);
    req.allowed_planning_time = allowed_planning_time;
    req.num_planning_attempts = replan_times;
    req.max_acceleration_scaling_factor = max_acceleration_scaling_factor;
    req.max_velocity_scaling_factor = max_velocity_scaling_factor;

    ROS_INFO_STREAM_NAMED(getName(), getName()<<": Compute path using planner: " << planner_id);
    ROS_DEBUG_STREAM_NAMED(getName(), getName()<<": start_state: " << req.start_state);
    ROS_DEBUG_STREAM_NAMED(getName(), getName()<<": group_name: " << req.group_name);
    ROS_DEBUG_STREAM_NAMED(getName(), getName()<<": planner_id: " << req.planner_id);
    ROS_DEBUG_STREAM_NAMED(getName(), getName()<<": allowed_planning_time: " << req.allowed_planning_time);
    ROS_DEBUG_STREAM_NAMED(getName(), getName()<<": num_planning_attempts: " << req.num_planning_attempts);
    ROS_DEBUG_STREAM_NAMED(getName(), getName()<<": max_velocity_scaling_factor: " << req.max_velocity_scaling_factor);
    ROS_DEBUG_STREAM_NAMED(getName(), getName()<<": goal_constraints: " << req.goal_constraints.at(0));

    motion_plan_client_.call(motion_plan);
      
    if(res.error_code.val == res.error_code.SUCCESS)
    {
      plan.start_state_ = res.trajectory_start;;
      plan.trajectory_ = res.trajectory;
      plan.planning_time_ = res.planning_time;
      error_code.val = moveit::planning_interface::MoveItErrorCode::SUCCESS;
    }
    else
      error_code.val = moveit::planning_interface::MoveItErrorCode::FAILURE;

    return error_code;
}

// plan with Cartesian Paths
moveit_msgs::MoveItErrorCodes ArmComputePathSkill::createPlanInterface(const geometry_msgs::PoseStamped& pose_target, 
                                                                        float eef_step,
                                                                        float jump_threshold,
                                                                        int32_t replan_times,
                                                                        moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    ROS_INFO_STREAM_NAMED(getName(), getName()<<": Compute path using Cartesian path");

    moveit_msgs::MoveItErrorCodes error_code;
    error_code.val = moveit::planning_interface::MoveItErrorCode::FAILURE;

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = 0;
    int trytimes = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    // move_group_->setStartState(*move_group_->getCurrentState());
    move_group_->setStartStateToCurrentState();
    waypoints.push_back(move_group_->getCurrentPose().pose);
    waypoints.push_back(pose_target.pose); 

    bool success = false;
    while (fraction <= 0.5 && trytimes < replan_times_)
    {
      fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      AdjustTrajectoryToFixTimeSequencing(trajectory);
      trytimes ++;
    }
  
    if (fraction < 0.5)
      {
        error_code.val = moveit::planning_interface::MoveItErrorCode::FAILURE;
        return error_code;
      }
      else
      {
        plan.trajectory_ = trajectory;
        plan.start_state_ = robot_state_;
        // Todo get planning time
        plan.planning_time_ = 0.0;
        error_code.val = moveit::planning_interface::MoveItErrorCode::SUCCESS;
        return error_code;
      }
}

// plan with name
moveit_msgs::MoveItErrorCodes ArmComputePathSkill::createPlanInterface(std::string &pose_name, 
                                                                        std::string& planner_id,
                                                                        float max_velocity_scaling_factor,
                                                                        float max_acceleration_scaling_factor,
                                                                        int32_t replan_times,
                                                                        float allowed_planning_time,
                                                                        moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    ROS_INFO_STREAM_NAMED(getName(), getName()<<": Compute path by Named goal using planner: " << planner_id);

    moveit_msgs::MoveItErrorCodes error_code;

    move_group_->setPlanningTime(allowed_planning_time);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    move_group_->setNumPlanningAttempts(replan_times);
    move_group_->setNamedTarget(pose_name);
    move_group_->setStartStateToCurrentState();
    move_group_->setPlannerId(planner_id);

    error_code = move_group_->plan(plan);

    move_group_->clearPathConstraints();
    return error_code;
}

void ArmComputePathSkill::AdjustTrajectoryToFixTimeSequencing(moveit_msgs::RobotTrajectory &trajectory)
{

  std::vector<ros::Duration> times_from_start;
  times_from_start.resize(trajectory.joint_trajectory.points.size());

  for(int i=0; i < times_from_start.size() ; i++)
  {
    times_from_start[i]= trajectory.joint_trajectory.points[i].time_from_start;
  }

  bool adjusted_flag=false;
  for(int i=1; i< times_from_start.size()-1;i++)
  {
    if(times_from_start[i]==ros::Duration(0))
    {
      ros::Duration prev = times_from_start[i];
      times_from_start[i] = ros::Duration((times_from_start[i-1].toSec()+times_from_start[i+1].toSec())/2.0);
      // ROS_WARN_STREAM("Recomputing point " << i << " from " << prev <<  " to: " << times_from_start[i-1] << " + " << times_from_start[i+1] << " = " <<times_from_start[i]);
      adjusted_flag=true;
    }
  }

  if( times_from_start.size()>1 &&  times_from_start[times_from_start.size()-1] == ros::Duration(0))
  {
    ROS_WARN_STREAM("Final point in trajectory has 0 timestamp, incrementing logically");
    times_from_start[times_from_start.size()-1] = times_from_start[times_from_start.size()-2] + ros::Duration(0.1);
    adjusted_flag=true;
  }

  if(adjusted_flag)
  {
    for(int i=0; i< times_from_start.size(); i++)
    {
      trajectory.joint_trajectory.points[i].time_from_start = times_from_start[i];
    }
  }
}

} // namespace