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

#include "moveit_skills/service/set_path_constrains.hpp"

namespace moveit_skills
{
SetPathConstrainsServiceServer::SetPathConstrainsServiceServer(
  const std::string & service_name, const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode(service_name, "", rclcpp::NodeOptions()), service_name_(service_name)
{
  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(this->get_name()));
}

SetPathConstrainsServiceServer::~SetPathConstrainsServiceServer() {}

nav2_util::CallbackReturn SetPathConstrainsServiceServer::on_configure(
  const rclcpp_lifecycle::State &)
{
  // Create the action server that perform some functions
  auto node = shared_from_this();
  service_server_ = node->create_service<ServiceT>(
    service_name_, std::bind(
                     &SetPathConstrainsServiceServer::execution, this, std::placeholders::_1,
                     std::placeholders::_2, std::placeholders::_3));
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn SetPathConstrainsServiceServer::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(*logger_, "Activating...");
  // create bond connection
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

void SetPathConstrainsServiceServer::execution(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<ServiceT::Request> request,
  const std::shared_ptr<ServiceT::Response> response)
{
  if (request->file_path.empty()) {
    response->error_code_id = response->INVALID_INPUT;
    return;
  }
  moveit_msgs::msg::Constraints path_constraints;
  if (loadPathConstraintsFromYaml(request->file_path, path_constraints)) {
    response->path_constraints = path_constraints;
    response->error_code_id = response->NONE;
  } else {
    response->error_code_id = response->LOADING_FAIL;
    return;
  }
}

bool SetPathConstrainsServiceServer::loadPathConstraintsFromYaml(
  const std::string path_constraints_yaml, moveit_msgs::msg::Constraints & path_constraints)
{
  YAML::Node config = YAML::LoadFile(path_constraints_yaml);
  auto doc = config["path_constraints"];
  try {
    auto path_constraint_name = getValueFromYaml<std::string>(doc, "name");
    if (path_constraint_name.empty()) {
      std::stringstream ss;
      ss << "The name tag was empty in '" << path_constraints_yaml << "' file ";
      RCLCPP_ERROR(*logger_, "%s", ss.str().c_str());
      return false;
    }
    path_constraints.name = path_constraint_name;
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "The name tag was undefined in '" << path_constraints_yaml << "' file ";
    RCLCPP_ERROR(*logger_, "%s", ss.str().c_str());
    return false;
  }

  try {
    for (const auto joint_doc : doc["joint_constraints"]) {
      moveit_msgs::msg::JointConstraint joint_constraint;
      joint_constraint.joint_name = getValueFromYaml<std::string>(joint_doc, "joint_name");
      joint_constraint.tolerance_above = getValueFromYaml<double>(joint_doc, "tolerance_above");
      joint_constraint.tolerance_below = getValueFromYaml<double>(joint_doc, "tolerance_below");
      joint_constraint.weight = getValueFromYaml<double>(joint_doc, "weight");

      path_constraints.joint_constraints.push_back(joint_constraint);
    }
  } catch (YAML::Exception & e) {
    RCLCPP_WARN(*logger_, "joint_constraints aren't defined in %s", path_constraints_yaml.c_str());
  }
  return true;
  // RWU TODO: parser PositionConstraint OrientationConstraint  VisibilityConstraint
}

template <typename T>
T SetPathConstrainsServiceServer::getValueFromYaml(const YAML::Node & node, const std::string & key)
{
  try {
    return node[key].as<T>();
  } catch (YAML::Exception & e) {
    std::stringstream ss;
    ss << "Failed to parse YAML tag '" << key << "' for reason: " << e.msg;
    throw YAML::Exception(e.mark, ss.str());
  }
}

}  // namespace moveit_skills
