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

#ifndef MOVEIT_SKILLS__GET_CURRENT_IK_FRAME_POSE_ACTION_SERVER_HPP_
#define MOVEIT_SKILLS__GET_CURRENT_IK_FRAME_POSE_ACTION_SERVER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit_skills/action/action_server_utils.hpp"
#include "moveit_skills/action/get_current_ik_frame_pose.hpp"
#include "moveit_skills/action/utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace moveit_skills
{
class GetCurrentIKFramePoseActionServer
: public ActionServerUtils<moveit_skills::action::GetCurrentIKFramePose>
{
public:
  using ActionT = moveit_skills::action::GetCurrentIKFramePose;

  GetCurrentIKFramePoseActionServer(
    const nav2_util::LifecycleNode::WeakPtr & parent, const std::string & action_name,
    planning_scene_monitor::PlanningSceneMonitorPtr psm,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~GetCurrentIKFramePoseActionServer();

  void execution() override;

  bool getIKFramePose(
    const std::string & group_name, const std::string & ik_frame_id,
    planning_scene_monitor::PlanningSceneMonitorPtr psm, geometry_msgs::msg::Pose & result);

protected:
  std::string action_name_;
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;
  std::shared_ptr<RosParameters> parameters_;
};

}  // namespace moveit_skills

#endif  // MOVEIT_SKILLS__GET_CURRENT_IK_FRAME_POSE_ACTION_SERVER_HPP_
