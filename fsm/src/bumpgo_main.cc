#include <cstdio>
#include <bumpgo/bumpgo_node.hpp>
#include <rclcpp/executors.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto bump_node = std::make_shared<fsm::bump_go::BumpGoNode>("bump_node");

  rclcpp::spin(bump_node);

  rclcpp::shutdown();
  return 0;
}
