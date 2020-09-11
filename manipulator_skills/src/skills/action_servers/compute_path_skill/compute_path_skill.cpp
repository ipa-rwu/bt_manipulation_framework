#include "manipulator_skills/skills/compute_path_skill.hpp"
#include <manipulator_skills/skill_names.hpp>

namespace manipulator_skills
{
ArmComputePathSkill::ArmComputePathSkill(std::string group_name) :
    ManipulatorSkill(COMPUTE_PATH_NAME),
    action_name_(COMPUTE_PATH_NAME),
    group_name_(group_name)
{
  // compute_path_action_server_ = new ComputePathActionServer(ros::NodeHandle(), COMPUTE_PATH_NAME, 
  // boost::bind(&ArmComputePathSkill::executeCB, this, _1), false);

  // compute_path_action_server_->start();
  this->initialize();
}

ArmComputePathSkill::~ArmComputePathSkill()
{
}

void ArmComputePathSkill::initialize()
{
    // move_group_ = new moveit::planning_interface::MoveGroupInterface(group_name_);
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));

    robot_model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>("robot_description");

    robot_model_ = robot_model_loader_->getModel();

    motion_plan_client_ = nh_.serviceClient<moveit_msgs::GetMotionPlan>("plan_kinematic_path");

    // planning_scene_ = std::make_unique<planning_scene::PlanningScene>(robot_model_);
    // planning_pipeline_ = std::make_unique<planning_pipeline::PlanningPipeline>(robot_model_, nh_, "planning_plugin", "request_adapters");

    // planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));

    // start the move action server
    as_.reset(new ComputePathActionServer(root_node_handle_, COMPUTE_PATH_NAME, 
    boost::bind(&ArmComputePathSkill::executeCB, this, _1), false));

    // robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    
    as_->start();
    ROS_INFO_STREAM_NAMED(getName(), "start action" );
}

void ArmComputePathSkill::executeCB(const man_msgs::ComputePathSkillGoalConstPtr& goal)
{
    // update robot state
    if(goal->is_attached)
    {
      // attach object to robot
    }
    else
    {
      current_state_ = move_group_->getCurrentState();
      robot_state::robotStateToRobotStateMsg(*current_state_ ,robot_state_);
    }

    // add construct
    end_effector_ = goal->end_effector;
    ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] end_effector is " << end_effector_);
    group_name_ = goal->group_name;
    planner_id_ = goal->planner_id;
    replan_times_ = goal->num_planning_attempts;
    pose_goal_ = goal->goal;
    max_acceleration_scaling_factor_ = (float) goal->max_acceleration_scaling_factor;
    max_velocity_scaling_factor_ = (float) goal->max_velocity_scaling_factor;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::MoveItErrorCodes error_code;
    // compute path with pose
    if(goal->target_type.compare("Pose") == 0)
  {    
      pose_constraints_ = updateGoal(position_tolerances_, orientation_tolerances_, goal->goal, end_effector_);

      ROS_INFO_NAMED(getName(), "[ComputePathSkill] max_velocity_scaling_factor is %f", goal->max_velocity_scaling_factor);
      // ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] pose of goal is " << goal->goal);

      current_state_ = move_group_->getCurrentState();
      robot_state::robotStateToRobotStateMsg(*current_state_ ,robot_state_);

      error_code = createPlanWithPipeline(pose_constraints_, 
                                          group_name_, 
                                          planner_id_,
                                          goal->max_velocity_scaling_factor,
                                          replan_times_, 
                                          goal->allowed_planning_time,
                                          robot_state_, 
                                          plan);
    }

    // compute path with pose name
    if(goal->target_type.compare("Name") == 0)
    {

      ROS_INFO_NAMED(getName(), "[ComputePathSkill] max_velocity_scaling_factor is %f", goal->max_velocity_scaling_factor);
      ROS_INFO_NAMED(getName(), "[ComputePathSkill] pose name of goal: \"%s\" ", goal->named_goal.c_str());

      std::string named_goal = goal->named_goal;
      error_code = createPlanInterface(named_goal, 
                                      planner_id_,
                                      goal->max_velocity_scaling_factor,
                                      goal->num_planning_attempts, 
                                      goal->allowed_planning_time,
                                      robot_state_, 
                                      plan);
    }


    // compute path with Cartesian Paths
    if(goal->target_type.compare("Cartesian") == 0)
    {
      current_state_ = move_group_->getCurrentState();
      robot_state::robotStateToRobotStateMsg(*current_state_ ,robot_state_);

        ROS_INFO_NAMED(getName(), "[ComputePathSkill] max_velocity_scaling_factor is %f", goal->max_velocity_scaling_factor);
        ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] pose of goal is " << goal->goal);

      error_code = createPlanInterface(goal->goal, 
                            goal->eef_step, 
                            goal->jump_threshold,
                            goal->num_planning_attempts, 
                            robot_state_, 
                            plan);
    }

    if (error_code.val == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      // action_res_.trajectory_start = plan.start_state_;
      // action_res_.trajectory = plan.trajectory_;
      // action_res_.planning_time = plan.planning_time_;
      // action_res_.group_name = goal->group_name;
      action_res_.plan.start_state = plan.start_state_;
      action_res_.plan.trajectory = plan.trajectory_;
      action_res_.plan.planning_time = plan.planning_time_;

      ROS_INFO_NAMED(getName(), "[ComputePathSkill] Compute path succeeded!");
      // ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] get path " << action_res_.plan.trajectory);


      const std::string response = "SUCCESS";
      as_->setSucceeded(action_res_, response);
    }
    else
    {
      ROS_INFO_NAMED(getName(), "[ComputePathSkill] Compute path Failed!");

      const std::string response = "FAILURE";
      as_->setAborted(action_res_, response);
    }
}

    // else if (service_res.error_code.val == service_res.error_code.PREEMPTED)
    // {
    //     const std::string response = "Preempted";
    //     as_->setPreempted(action_res, response);
    // }


// bool ArmComputePathSkill::update_robot_state(bool is_attched)
// {

// }
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
moveit_msgs::MoveItErrorCodes ArmComputePathSkill::createPlanWithPipeline(moveit_msgs::Constraints& pose_constraints,
                                                  std::string& group_name,
                                                  std::string& planner_id,
                                                  float max_velocity_scaling_factor,
                                                  int32_t replan_times,
                                                  float allowed_planning_time,
                                                  const moveit_msgs::RobotState& start_robot_state,
                                                  moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    ROS_INFO_NAMED(getName(), "[ComputePathSkill] createPlanWithPipeline for Arm request received");

    moveit_msgs::MoveItErrorCodes error_code;
    error_code.val = moveit::planning_interface::MoveItErrorCode::FAILURE;

    // creating motion plan request
    moveit_msgs::GetMotionPlan motion_plan;
    moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
    moveit_msgs::MotionPlanResponse &res = motion_plan.response.motion_plan_response;

    // planning_interface::MotionPlanRequest req;
    // planning_interface::MotionPlanResponse res;

    // current_state_ = move_group_->getCurrentState();
    // robot_state::robotStateToRobotStateMsg(*current_state_ ,robot_state_);

    req.goal_constraints.clear();

    req.start_state = robot_state_;
    // req.start_state.is_diff = true;
    req.group_name = group_name;
    req.planner_id = planner_id;
    req.goal_constraints.push_back(pose_constraints);
    req.allowed_planning_time = allowed_planning_time;
    req.num_planning_attempts = replan_times;
    req.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;
    req.max_velocity_scaling_factor = max_velocity_scaling_factor_;

    // planning_scene_->setCurrentState(robot_state_);

    // // Now, call the pipeline and check whether planning was successful.
    // planning_pipeline_->generatePlan(planning_scene_, req, res);

    ROS_INFO_NAMED(getName(), "[ComputePathSkill] createPlanWithPipeline for Arm request: ");
    ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] start_state: " << req.start_state);
    ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] group_name: " << req.group_name);
    ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] planner_id: " << req.planner_id);
    ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] allowed_planning_time: " << req.allowed_planning_time);
    ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] num_planning_attempts: " << req.num_planning_attempts);
    ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] max_velocity_scaling_factor: " << req.max_velocity_scaling_factor);
    ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] goal_constraints: " << req.goal_constraints.at(0));

    ROS_INFO_NAMED(getName(), "[ComputePathSkill] call motionplan");

    if(motion_plan_client_.call(motion_plan))
    { 
      ROS_INFO_NAMED(getName(), "[ComputePathSkill] call motionplan Succeeded");
      ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] call motionplan result: " << res.error_code.val);
    }
    else
    {
      ROS_INFO_NAMED(getName(), "[ComputePathSkill] call motionplan Failed");
    }
      
      if(res.error_code.val == res.error_code.SUCCESS)
      {
        // moveit_msgs::MotionPlanResponse response;
        // res.getMessage(response);
        
        plan.start_state_ = res.trajectory_start;;
        plan.trajectory_ = res.trajectory;
        plan.planning_time_ = res.planning_time;
        error_code.val = moveit::planning_interface::MoveItErrorCode::SUCCESS;
        ROS_INFO_NAMED(getName(), "[ComputePathSkill] createPlanWithPipeline for Arm Succeeded");

      }
      else
        error_code.val = moveit::planning_interface::MoveItErrorCode::FAILURE;
        ROS_INFO_STREAM_NAMED(getName(), "[ComputePathSkill] createPlanWithPipeline : " << res.error_code.val);

    return error_code;
}

// plan with Cartesian Paths
moveit_msgs::MoveItErrorCodes ArmComputePathSkill::createPlanInterface(const geometry_msgs::PoseStamped& pose_target, 
                                                                        float eef_step,
                                                                        float jump_threshold,
                                                                        int32_t replan_times,
                                                                        const moveit_msgs::RobotState &start_robot_state,
                                                                        moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    ROS_INFO_NAMED(getName(), "[ComputePathSkill] Compute Path for Cartesian Paths");

    moveit_msgs::MoveItErrorCodes error_code;
    error_code.val = moveit::planning_interface::MoveItErrorCode::FAILURE;

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = 0;
    int trytimes = 0;
    std::vector<geometry_msgs::Pose> waypoints;

    move_group_->setStartState(*move_group_->getCurrentState());

    waypoints.push_back(move_group_->getCurrentPose().pose);
    waypoints.push_back(pose_target.pose); 

    bool success = false;
    while (fraction < 0.5 && trytimes < replan_times_)
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
                                                                        int32_t replan_times,
                                                                        float allowed_planning_time,
                                                                        const moveit_msgs::RobotState &start_robot_state,
                                                                        moveit::planning_interface::MoveGroupInterface::Plan& plan)
{
    ROS_INFO_NAMED(getName(), "[ComputePathSkill] Compute Path for pose name: \"%s\" ", pose_name.c_str());

    moveit_msgs::MoveItErrorCodes error_code;

    move_group_->setPlanningTime(allowed_planning_time);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
    move_group_->setNumPlanningAttempts(replan_times);
    move_group_->setNamedTarget(pose_name);
    move_group_->setStartStateToCurrentState();
    // move_group_->setStartState(start_robot_state);
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
      // ROS_INFO_STREAM("Recomputed time point " << i << " : " << trajectory.joint_trajectory.points[i].time_from_start );
    }
  }

}

} // namespace