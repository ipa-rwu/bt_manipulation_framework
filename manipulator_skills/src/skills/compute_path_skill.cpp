#include "manipulator_skills/skills/compute_path_skill.h"
#include <manipulator_skills/skill_names.h>

namespace manipulator_skills
{
ArmComputePathSkill::ArmComputePathSkill(std::string name) :
    ManipulatorSkill(COMPUTE_PATH_NAME),
    action_name_(COMPUTE_PATH_NAME),
    as_(NULL)
  {
    // compute_path_action_server_ = new ComputePathActionServer(ros::NodeHandle(), COMPUTE_PATH_NAME, 
    // boost::bind(&ArmComputePathSkill::executeCB, this, _1), false);

    // compute_path_action_server_->start();
    this->initialize();
  }

ArmComputePathSkill::~ArmComputePathSkill()
{
    if(as_ != NULL)
        delete as_;
}

void ArmComputePathSkill::initialize()
{
    // start the move action server
    as_ = new ComputePathActionServer(root_node_handle_, COMPUTE_PATH_NAME, 
    boost::bind(&ArmComputePathSkill::executeCB, this, _1), false);

    as_->start();
    ROS_INFO_STREAM_NAMED(getName(), "start action" );
}

void ArmComputePathSkill::executeCB(const man_msgs::ComputePathSkillGoalConstPtr& goal)
{
  man_msgs::ComputePathSkillResult action_res;
  moveit_msgs::MotionPlanResponse service_res;
  ROS_INFO_STREAM_NAMED(goal->pose.header.frame_id, "frame_id");
  if (computePath(goal, action_res, service_res))
  {
    if (service_res.error_code.val == service_res.error_code.SUCCESS)
    {
        const std::string response = "success";
        action_res.trajectory_start = service_res.trajectory_start;
        action_res.trajectory = service_res.trajectory;
        action_res.planning_time = service_res.planning_time;
        as_->setSucceeded(action_res, response);

    }
    else if (service_res.error_code.val == service_res.error_code.FAILURE)
    {
        const std::string response = "FAILURE";
        as_->setAborted(action_res, response);
    }
    else if (service_res.error_code.val == service_res.error_code.PREEMPTED)
    {
        const std::string response = "Preempted";
        as_->setPreempted(action_res, response);
    }
  }
}

bool ArmComputePathSkill::computePath(const man_msgs::ComputePathSkillGoalConstPtr& goal,
                                        man_msgs::ComputePathSkillResult& action_res,
                                        moveit_msgs::MotionPlanResponse &res)
{
    ROS_INFO_NAMED(getName(), "Compute Path for Arm request received");

    // constructing motion plan goal constraints
//   std::vector<double> position_tolerances(3,0.01f);
//   std::vector<double> orientation_tolerances(3,0.01f);
//   geometry_msgs::PoseStamped p;
//   p.header.frame_id = cfg.WORLD_FRAME_ID;
//   p.pose = pose_target;

    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(goal->pose.header.frame_id, goal->pose, goal->position_tolerances,
                                         goal->orientation_tolerances);

    // creating motion plan request
    moveit_msgs::GetMotionPlan motion_plan;
    moveit_msgs::MotionPlanRequest &req = motion_plan.request.motion_plan_request;
    res = motion_plan.response.motion_plan_response;

    req.start_state = goal->start_state;
    req.start_state.is_diff = true;
    req.group_name = goal->group_name;
    req.goal_constraints.push_back(pose_goal);
    req.allowed_planning_time = goal->allowed_planning_time;
    req.num_planning_attempts = goal->num_planning_attempts;
    req.max_velocity_scaling_factor = goal->max_velocity_scaling_factor;
    req.max_acceleration_scaling_factor = goal->max_acceleration_scaling_factor;

    // request motion plan
    bool success = motion_plan_client_.call(motion_plan);
    // if(motion_plan_client_.call(motion_plan) && res.error_code.val == res.error_code.SUCCESS)
    // {
    //     // saving motion plan results
    //     action_res.start_state = res.trajectory_start;
    //     action_res.trajectory = res.trajectory;
    //     action_res.planning_time = res.planning_time;
    //     success = true;
    // }
    return success;
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

