#include <memory>

#include "man2_bt_operator/bt_operator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<man2_bt_operator::BTOperator>();

  rclcpp::executors::MultiThreadedExecutor exe;

  exe.add_node(node->get_node_base_interface());

  exe.spin();

  rclcpp::shutdown();

  return 0;
}
