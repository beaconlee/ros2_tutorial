#include <cstdio>
#include <rclcpp/rclcpp.hpp>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // 如果没有调用 rclcpp::Init() 函数的话，会报错如下错误
  // terminate called after throwing an instance of 'rclcpp::exceptions::RCLInvalidArgument'
  // what():  failed to create guard condition: context argument is null, at ./src/rcl/guard_condition.c:65
  // [ros2run]: Aborted

  // create ros2 node, node is a std::shared_ptr<rclcpp::Node> to ros2 node whose named "basic_node";

  auto node = rclcpp::Node::make_shared("basic_node");
  RCLCPP_INFO(node->get_logger(), "This is my first Node in ros2.");


  auto node2 = std::make_shared<rclcpp::Node>("auto_make_shared_node");
  RCLCPP_INFO(node2->get_logger(), "Use make_shared construct node");

  // rclcpp::Node::SharedPtr is an aliys for std::shared_ptr<rclcpp::Node>
  rclcpp::Node::SharedPtr node3 =
      std::make_shared<rclcpp::Node>("type_make_shared_node");
  RCLCPP_INFO(node3->get_logger(), "Use make_shared construct node");

  auto node4 = std::shared_ptr<rclcpp::Node>(new rclcpp::Node("new_node"));
  RCLCPP_INFO(node4->get_logger(), "Use make_shared construct node");


  // rclcpp::spin(node);
  // rclcpp::spin(node2);

  rclcpp::shutdown();

  return 0;
}
