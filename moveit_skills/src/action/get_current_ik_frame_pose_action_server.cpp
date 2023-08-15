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

#include "moveit_skills/action/get_current_ik_frame_pose_action_server.hpp"

namespace moveit_skills
{
GetCurrentIKFramePoseActionServer::GetCurrentIKFramePoseActionServer(
  const nav2_util::LifecycleNode::WeakPtr & parent, const std::string & action_name,
  planning_scene_monitor::PlanningSceneMonitorPtr psm, const rclcpp::NodeOptions & /*options*/)
: ActionServerUtils<moveit_skills::action::GetCurrentIKFramePose>(),
  action_name_(action_name),
  psm_(psm)
{
  auto node = parent.lock();
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(node->get_name() + action_name_));

  parameters_ = std::make_shared<RosParameters>(node);
  parameters_->loadRosParameters(node);

  psm_->providePlanningSceneService(parameters_->get_planning_scene_service_name);

  action_server_ = std::make_unique<ActionServer>(
    node, action_name_, std::bind(&GetCurrentIKFramePoseActionServer::execution, this), nullptr,
    std::chrono::milliseconds(500), true);
}

GetCurrentIKFramePoseActionServer::~GetCurrentIKFramePoseActionServer() {}

void GetCurrentIKFramePoseActionServer::execution()
{
  auto goal = getCurrentGoal();
  auto result = std::make_shared<ActionT::Result>();

  if (getIKFramePose(goal->group_name, goal->ik_frame, psm_, result->pose)) {
    result->error_code_id = result->NONE;
    sendSucceededResult(result);
  }
  result->error_code_id = result->CALCULATION_FAIL;
  sendFaildResult(result);
}

bool GetCurrentIKFramePoseActionServer::getIKFramePose(
  const std::string & group_name, const std::string & ik_frame_id,
  planning_scene_monitor::PlanningSceneMonitorPtr psm, geometry_msgs::msg::Pose & result)
{
  const moveit::core::JointModelGroup * jmg =
    planning_scene_monitor::LockedPlanningSceneRO(psm)->getRobotModel()->getJointModelGroup(
      group_name);
  if (!jmg) {
    RCLCPP_ERROR(*logger_, "Could not joint group from %s", group_name.c_str());
    return false;
  }

  const planning_scene::PlanningScenePtr lscene = [&] {
    planning_scene_monitor::LockedPlanningSceneRO ls(psm);
    return planning_scene::PlanningScene::clone(ls);
  }();

  auto current_robot_state = lscene->getCurrentState();

  Eigen::Isometry3d target;

  // tip link
  const moveit::core::LinkModel * link;
  // the translation vector from link to global(planning) frame
  Eigen::Isometry3d ik_pose_world;
  if (!getRobotTipForFrame(lscene, jmg, link, ik_pose_world, ik_frame_id)) return false;

  result = tf2::toMsg(current_robot_state.getGlobalLinkTransform(link));
  RCLCPP_DEBUG(
    *logger_, "Current pose of %s: %s", link->getName().c_str(),
    geometry_msgs::msg::to_yaml(result).c_str());
  return true;
}

}  // namespace moveit_skills
