#include "manipulator_skills/skills/compute_path_skill.hpp"
#include <manipulator_skills/skill_names.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


namespace manipulator_skills
{
ArmComputePathSkill::ArmComputePathSkill(std::string group_name) :
    ManipulatorSkill(COMPUTE_PATH_NAME),
    action_name_(COMPUTE_PATH_NAME),
    group_name_(group_name),
    isCartesianPath_(false)
    {
    // compute_path_action_server_ = new ComputePathActionServer(ros::NodeHandle(), COMPUTE_PATH_NAME, 
    // boost::bind(&ArmComputePathSkill::executeCB, this, _1), false);

    // compute_path_action_server_->start();
    this->initialize();
  }

ArmComputePathSkill::~ArmComputePathSkill()
{
    if(move_group_ != NULL)
    delete move_group_;
}

void ArmComputePathSkill::initialize()
{
    // move_group_ = new moveit::planning_interface::MoveGroupInterface(group_name_);
    move_group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
    // start the move action server
    as_.reset(new ComputePathActionServer(root_node_handle_, FIND_OBJECTS_NAME, 
    boost::bind(&ArmComputePathSkill::executeCB, this, _1), false));
    
    as_->start();
    ROS_INFO_STREAM_NAMED(getName(), "start action" );
}

void ArmComputePathSkill::executeCB(const man_msgs::ComputePathSkillGoalConstPtr& goal)
{
  man_msgs::ComputePathSkillResult action_res;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit_msgs::MoveItErrorCodes error_code;


  if (computePath(goal, plan, error_code).val == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    action_res.trajectory_start = plan.start_state_;
    action_res.trajectory = plan.trajectory_;
    action_res.planning_time = plan.planning_time_;
    const std::string response = "SUCCESS";
    as_->setSucceeded(action_res, response);
  } 
  else if (computePath(goal, plan, error_code).val == moveit::planning_interface::MoveItErrorCode::FAILURE)
  {
    const std::string response = "FAILURE";
    as_->setAborted(action_res, response);
  }

    // else if (service_res.error_code.val == service_res.error_code.PREEMPTED)
    // {
    //     const std::string response = "Preempted";
    //     as_->setPreempted(action_res, response);
    // }
}

moveit_msgs::MoveItErrorCodes ArmComputePathSkill::computePath(const man_msgs::ComputePathSkillGoalConstPtr& goal,
                                         moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                         moveit_msgs::MoveItErrorCodes error_code)
{
    ROS_INFO_NAMED(getName(), "Compute Path for Arm request received");

    // constructing motion plan goal constraints
//   std::vector<double> position_tolerances(3,0.01f);
//   std::vector<double> orientation_tolerances(3,0.01f);
//   geometry_msgs::PoseStamped p;
//   p.header.frame_id = cfg.WORLD_FRAME_ID;
//   p.pose = pose_target;
    replan_times_ = goal->num_planning_attempts;
    moveit_msgs::RobotState start_state = goal->start_state;
    geometry_msgs::Pose target_pose = goal->pose;
    geometry_msgs::Pose current_pose = goal->current_pose;

    move_group_->setPlanningTime(goal->allowed_planning_time);
    move_group_->setMaxVelocityScalingFactor(goal->max_velocity_scaling_factor);
    move_group_->setNumPlanningAttempts(replan_times_);


    if (goal->target_type == "Name")
    {
      move_group_->setNamedTarget(goal->named_target);
      isCartesianPath_ = false;
    }

    if (goal->target_type == "Pose")
    {
      move_group_->setPoseTarget(target_pose);
      isCartesianPath_ = false;
    }

    if (goal->target_type == "CartesianPose")
    {
      isCartesianPath_ = true; 

    }

    if (isCartesianPath_ == false)
    {
      move_group_->setStartState(start_state);
      error_code = move_group_->plan(plan);
      return error_code;
    }

    else
    {
      moveit_msgs::RobotTrajectory trajectory;
      double fraction = 0;
      int trytimes = 0;
      std::vector<geometry_msgs::Pose> waypoints;
      waypoints.push_back(current_pose);
      waypoints.push_back(target_pose); 
      while (fraction < 0.5 && trytimes < replan_times_)
      {
        fraction = move_group_->computeCartesianPath(waypoints, goal->eef_step, goal->jump_threshold, trajectory);
        AdjustTrajectoryToFixTimeSequencing(trajectory);
        trytimes ++;
      }
    
    if (fraction < 0.5 && trytimes >= replan_times_)
      {
        error_code.val = moveit::planning_interface::MoveItErrorCode::FAILURE;
        return error_code;
      }
      else
      {
        plan.trajectory_ = trajectory;
        plan.start_state_ = goal->start_state;
        // Todo get planning time
        plan.planning_time_ = 0.0;
        return error_code;
      }

    }


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







// namespace manipulator_skills
// {
// ArmComputePathSkill::ArmComputePathSkill(std::string name):
//     compute_path_action_server_(nh_, name, 
//     boost::bind(&ArmComputePathSkill::computePathCallback, this, _1), false),
//     action_name_(name)
// {
//     name = "computePathServer";
//     compute_path_action_server_.start();
//     ROS_INFO("start action");
// }



// // void ArmComputePathSkill::initialize()
// // {
// //   // start the move action server
// // //   compute_path_action_server_(nh_, COMPUTE_PATH_NAME, 
// // //     boost::bind(&ArmComputePathSkill::computePathCallback, this, _1), false);
// // //   computer_path_action_server_->registerPreemptCallback(
// // //       boost::bind(&ArmComputerPathSkill::preemptExecuteTrajectoryCallback, this));
// //   compute_path_action_server_.start();
// //   ROS_INFO("start action");
// // }

// void ArmComputePathSkill::computePathCallback(const man_msgs::ComputePathSkillGoalConstPtr& goal)
// {
//   man_msgs::ComputePathSkillResult action_res;
//   moveit_msgs::MotionPlanResponse service_res;
//   if (computePath(goal, action_res, service_res))
//   {
//     if (service_res.error_code.val == service_res.error_code.SUCCESS)
//     {
//         const std::string response = "success";
//         compute_path_action_server_.setSucceeded(action_res, response);
//     }
//     else if (service_res.error_code.val == service_res.error_code.FAILURE)
//     {
//         const std::string response = "FAILURE";
//         compute_path_action_server_.setAborted(action_res, response);
//     }
//     else if (service_res.error_code.val == service_res.error_code.PREEMPTED)
//     {
//         const std::string response = "Preempted";
//         compute_path_action_server_.setPreempted(action_res, response);
//     }
//   }
// }

// bool ArmComputePathSkill::computePath(const man_msgs::ComputePathSkillGoalConstPtr& goal,
//                                         man_msgs::ComputePathSkillResult& action_res,
//                                         moveit_msgs::MotionPlanResponse &res)
// {
//     // ROS_INFO_NAMED(getName(), "Compute Path for Arm request received");

//     // constructing motion plan goal constraints
// //   std::vector<double> position_tolerances(3,0.01f);
// //   std::vector<double> orientation_tolerances(3,0.01f);
// //   geometry_msgs::PoseStamped p;
// //   p.header.frame_id = cfg.WORLD_FRAME_ID;
// //   p.pose = pose_target;

//     moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(goal->pose.header.frame_id, goal->pose, goal->position_tolerances,
//                                          goal->orientation_tolerances);

//     // creating motion plan request
//     moveit_msgs::GetMotionPlan motion_plan;
//     moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
//     res = motion_plan.response.motion_plan_response;

//     req.start_state = goal->start_state;
//     req.start_state.is_diff = true;
//     req.group_name = goal->group_name;
//     req.goal_constraints.push_back(pose_goal);
//     req.allowed_planning_time = goal->allowed_planning_time;
//     req.num_planning_attempts = goal->num_planning_attempts;

//     // request motion plan
//     bool success = motion_plan_client_.call(motion_plan);
//     // if(motion_plan_client_.call(motion_plan) && res.error_code.val == res.error_code.SUCCESS)
//     // {
//     //     // saving motion plan results
//     //     action_res.start_state = res.trajectory_start;
//     //     action_res.trajectory = res.trajectory;
//     //     action_res.planning_time = res.planning_time;
//     //     success = true;
//     // }
//     return success;
// }



// // void MoveGroupExecuteTrajectoryAction::preemptExecuteTrajectoryCallback()
// // {
// //   context_->trajectory_execution_manager_->stopExecution(true);
// // }

// // void MoveGroupExecuteTrajectoryAction::setExecuteTrajectoryState(MoveGroupState state)
// // {
// //   moveit_msgs::ExecuteTrajectoryFeedback execute_feedback;
// //   execute_feedback.state = stateToStr(state);
// //   execute_action_server_->publishFeedback(execute_feedback);
// // }

// } // namespace

