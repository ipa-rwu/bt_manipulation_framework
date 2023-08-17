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

#ifndef MOVEIT_SKILLS__SET_PATH_CONSTRAINTS_SERVICE_SERVER_HPP_
#define MOVEIT_SKILLS__SET_PATH_CONSTRAINTS_SERVICE_SERVER_HPP_

#include <yaml-cpp/yaml.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit_skills/srv/set_path_constrains.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace moveit_skills
{
class SetPathConstrainsServiceServer : public nav2_util::LifecycleNode

{
public:
  using ServiceT = moveit_skills::srv::SetPathConstrains;

  SetPathConstrainsServiceServer(
    const std::string & service_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~SetPathConstrainsServiceServer();

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &);

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(*logger_, "Deactivating...");
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(*logger_, "Cleaning up...");
    service_server_.reset();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(*logger_, "Shutting down...");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void execution(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<typename ServiceT::Request> request,
    const std::shared_ptr<typename ServiceT::Response> response);

  bool loadPathConstraintsFromYaml(
    const std::string path_constraints_yaml, moveit_msgs::msg::Constraints & path_constraints);

  template <typename T>
  T getValueFromYaml(const YAML::Node & node, const std::string & key);

protected:
  rclcpp::Service<ServiceT>::SharedPtr service_server_;
  std::string service_name_;
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace moveit_skills

#endif  // MOVEIT_SKILLS__SET_PATH_CONSTRAINTS_SERVICE_SERVER_HPP_
