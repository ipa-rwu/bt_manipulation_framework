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

#include "moveit_skills/moveit_skill_server.hpp"

namespace moveit_skills
{
MoveitSkillServer::MoveitSkillServer()
: nav2_util::LifecycleNode("moveit_skill_server", "", rclcpp::NodeOptions())
{
  auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args", "-r",
     std::string("__node:=") + std::string(this->get_name()) + "_" + "moveit_cpp_node", "-p",
     "use_sim_time:=" +
       std::string(this->get_parameter("use_sim_time").as_bool() ? "true" : "false"),
     "--"});
  options.automatically_declare_parameters_from_overrides(true);

  client_node_ = std::make_shared<rclcpp::Node>("_", options);

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(this->get_name()));
}

MoveitSkillServer::~MoveitSkillServer() {}

nav2_util::CallbackReturn MoveitSkillServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(*logger_, "Configuring...");

  auto node = shared_from_this();

  moveit_cpp::MoveItCpp::Options moveit_cpp_options(client_node_);
  moveit_cpp_ptr_ = std::make_shared<moveit_cpp::MoveItCpp>(client_node_, moveit_cpp_options);

  psm_ = moveit_cpp_ptr_->getPlanningSceneMonitor();

  compute_path_to_state_action_ = std::make_unique<ComputePathToStateActionServer>(
    shared_from_this(), "compute_path_to_state", moveit_cpp_ptr_, psm_);
  compute_path_to_point_action_ = std::make_unique<ComputePathToPointActionServer>(
    shared_from_this(), "compute_path_to_point", moveit_cpp_ptr_, psm_);

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveitSkillServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(*logger_, "Activate...");
  compute_path_to_state_action_->activate();
  compute_path_to_point_action_->activate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveitSkillServer::on_deactivate(
  const rclcpp_lifecycle::State & /*state*/)
{
  compute_path_to_state_action_->deactivate();
  compute_path_to_point_action_->deactivate();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveitSkillServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  client_node_.reset();
  moveit_cpp_ptr_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveitSkillServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  return nav2_util::CallbackReturn::SUCCESS;
}
}  // namespace moveit_skills
