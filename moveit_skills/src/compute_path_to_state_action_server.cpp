// Copyright (c) 2023 Ruichao Wu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "moveit_skills/compute_path_to_state_action_server.hpp"

namespace moveit_skills
{
ComputePathToStateActionServer::ComputePathToStateActionServer(
  const nav2_util::LifecycleNode::WeakPtr & parent, const std::string & action_name,
  moveit_cpp::MoveItCppPtr & moveit_cpp_ptr, planning_scene_monitor::PlanningSceneMonitorPtr psm,
  const rclcpp::NodeOptions & /*options*/)
: ComputePathActionServerCore<moveit_skills::action::ComputePathToState>(),
  action_name_(action_name),
  moveit_cpp_ptr_(moveit_cpp_ptr),
  psm_(psm)
{
  auto node = parent.lock();
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(node->get_name() + action_name_));

  parameters_ = std::make_shared<RosParameters>(node);
  parameters_->loadRosParameters(node);

  plan_params_ = std::make_shared<moveit_cpp::PlanningComponent::PlanRequestParameters>();
  initPlanComponentParameters(*plan_params_, *parameters_);

  psm_->providePlanningSceneService(parameters_->get_planning_scene_service_name);

  action_server_ = std::make_unique<ActionServer>(
    node, action_name_, std::bind(&ComputePathToStateActionServer::execution, this), nullptr,
    std::chrono::milliseconds(500), true);
}

ComputePathToStateActionServer::~ComputePathToStateActionServer() {}

void ComputePathToStateActionServer::initial() {}

void ComputePathToStateActionServer::execution()
{
  auto goal = getCurrentGoal();
  auto result = std::make_shared<ActionT::Result>();

  RCLCPP_INFO(
    *logger_, "from parameter: planner id: %s, planning pipeline: %s",
    parameters_->planner_id.c_str(), parameters_->planning_pipeline.c_str());

  RCLCPP_INFO(
    *logger_, "from goal: planner id: %s, planning pipeline: %s", goal->config.planner_id.c_str(),
    goal->config.planning_pipeline.c_str());

  RCLCPP_INFO(
    *logger_, "from plan_params: planner id: %s, planning pipeline: %s",
    plan_params_->planner_id.c_str(), plan_params_->planning_pipeline.c_str());

  initPlanComponentParameters(*plan_params_, *parameters_);

  setPlanner(*plan_params_, goal->config.planning_pipeline, goal->config.planner_id);

  robot_trajectory::RobotTrajectoryPtr robot_trajectory;
  if (computePath(
        moveit_cpp_ptr_, *plan_params_, goal->target_group, goal->named_state, psm_,
        parameters_->goal_joint_tolerance, parameters_->goal_orientation_tolerance,
        parameters_->goal_position_tolerance, robot_trajectory)) {
    trajectoryTimeParametrization(robot_trajectory, *plan_params_);

    robot_trajectory->getRobotTrajectoryMsg(result->plan_result.trajectory);
    result->plan_result.error_code_id = result->plan_result.NONE;
    sendSucceededResult(result);
  }
  result->plan_result.error_code_id = result->plan_result.PLANNING_FAIL;
  sendFaildResult(result);
}

}  // namespace moveit_skills
