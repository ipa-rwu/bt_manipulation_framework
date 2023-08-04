// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "man2_behavior_tree/bt_service_client_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;  // NOLINT
using namespace std::placeholders;     // NOLINT

/**
 * @brief create an action server
 *
 */
class AddServiceServer : public rclcpp::Node
{
public:
  AddServiceServer() : rclcpp::Node("add_service_node", rclcpp::NodeOptions()), sleep_duration_(0ms)
  {
    server_ = create_service<example_interfaces::srv::AddTwoInts>(
      "test_service", std::bind(&AddServiceServer::handle_service, this, _1, _2, _3));
  }

protected:
  virtual void handle_service(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    (void)request_header;
    (void)response;
    current_request_ = request;
    response->sum = current_request_->a + current_request_->b;
  }

protected:
  std::chrono::milliseconds sleep_duration_;

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr server_;
  std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> current_request_;
};

/**
 * @brief test bt service client node
 *
 */
class TestBtServiceClient
: public ros2_behavior_tree::BtServiceClientNode<example_interfaces::srv::AddTwoInts>
{
public:
  TestBtServiceClient(const std::string & service_client_name, const BT::NodeConfiguration & conf)
  : ros2_behavior_tree::BtServiceClientNode<example_interfaces::srv::AddTwoInts>(
      service_client_name, conf)
  {
  }

  void on_tick() override
  {
    getInput("a", request_->a);
    getInput("b", request_->b);
  }

  BT::NodeStatus on_completion(
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> /*response*/) override
  {
    config().blackboard->set<int>("sum", future_result_.get()->sum);
    return BT::NodeStatus::SUCCESS;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<int>("a"), BT::InputPort<int>("b")});
  }
};

class BTServiceClientNodeTestFixture : public ::testing::Test
{
public:
  static void SetUpTestCase()
  {
    node_ = std::make_shared<rclcpp::Node>("bt_service_client_node_test");
    factory_ = std::make_shared<BT::BehaviorTreeFactory>();

    config_ = new BT::NodeConfiguration();

    // Create the blackboard that will be shared by all of the nodes in the tree
    config_->blackboard = BT::Blackboard::create();
    // Put items on the blackboard
    config_->blackboard->set<rclcpp::Node::SharedPtr>("node", node_);
    config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
    config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
    config_->blackboard->set<int>("a", 1);
    config_->blackboard->set<int>("b", 1);
    config_->blackboard->set<std::string>("service_name", "service name");

    factory_->registerBuilder(
      BT::CreateManifest<TestBtServiceClient>(
        "TestBTServiceClient", TestBtServiceClient::providedPorts()),
      BT::CreateBuilder<TestBtServiceClient>());
  }

  static void TearDownTestCase()
  {
    delete config_;
    config_ = nullptr;
    node_.reset();
    service_server_.reset();
    factory_.reset();
  }

  void SetUp() override
  {
    // initialize action server and spin on new thread
    service_server_ = std::make_shared<AddServiceServer>();
    server_thread_ = std::make_shared<std::thread>([]() {
      while (rclcpp::ok() && BTServiceClientNodeTestFixture::service_server_ != nullptr) {
        rclcpp::spin_some(BTServiceClientNodeTestFixture::service_server_);
        std::this_thread::sleep_for(100ns);
      }
    });
  }

  void TearDown() override
  {
    service_server_.reset();
    tree_.reset();
    server_thread_->join();
    server_thread_.reset();
  }

  static std::shared_ptr<AddServiceServer> service_server_;

protected:
  static rclcpp::Node::SharedPtr node_;
  static BT::NodeConfiguration * config_;
  static std::shared_ptr<BT::BehaviorTreeFactory> factory_;
  static std::shared_ptr<BT::Tree> tree_;
  static std::shared_ptr<std::thread> server_thread_;
};

rclcpp::Node::SharedPtr BTServiceClientNodeTestFixture::node_ = nullptr;
std::shared_ptr<AddServiceServer> BTServiceClientNodeTestFixture::service_server_ = nullptr;
BT::NodeConfiguration * BTServiceClientNodeTestFixture::config_ = nullptr;
std::shared_ptr<BT::BehaviorTreeFactory> BTServiceClientNodeTestFixture::factory_ = nullptr;
std::shared_ptr<BT::Tree> BTServiceClientNodeTestFixture::tree_ = nullptr;
std::shared_ptr<std::thread> BTServiceClientNodeTestFixture::server_thread_ = nullptr;

TEST_F(BTServiceClientNodeTestFixture, test_server_timeout_success)
{
  // create tree
  std::string xml_txt =
    R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
            <TestBTServiceClient service_name="test_service" a="41" b="1" />
        </BehaviorTree>
      </root>)";

  // the server timeout is larger than the goal handling duration
  config_->blackboard->set<std::chrono::milliseconds>("server_timeout", 20ms);
  config_->blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);

  auto bb = config_->blackboard;
  for (auto entry_name : bb->getKeys()) {
    std::string name(entry_name);
    std::cout << " key: " << name << std::endl;
  }

  tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(xml_txt, config_->blackboard));

  // to keep track of the number of ticks it took to reach a terminal result
  int ticks = 0;

  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // BT loop execution rate
  rclcpp::WallRate loopRate(10ms);

  // main BT execution loop
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_->tickWhileRunning();
    ticks++;
    loopRate.sleep();
  }

  // get calculated sum from blackboard
  auto sum = config_->blackboard->get<int>("sum");

  EXPECT_EQ(result, BT::NodeStatus::SUCCESS);
  EXPECT_EQ(sum, 42);

  // halt BT for a new execution cycle
  tree_->haltTree();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
