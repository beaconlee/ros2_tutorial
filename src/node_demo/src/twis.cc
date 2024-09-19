#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class TwistPublisher : public rclcpp::Node
{
public:
  TwistPublisher()
    : Node("twist_publisher")
  {
    // Create a publisher for the Twist message
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",
                                                          rclcpp::QoS(10));

    // Set up a timer to periodically publish Twist messages
    timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                     [this]() { publish_twist(); });
  }

private:
  void
  publish_twist()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.5;  // Example linear velocity
    message.angular.z = 0.1; // Example angular velocity
    // message.header.stamp = this->get_clock()->now();


    RCLCPP_INFO(this->get_logger(),
                "Publishing: linear.x: '%f', angular.z: '%f'",
                message.linear.x,
                message.angular.z);
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int
main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPublisher>());
  rclcpp::shutdown();
  return 0;
}
