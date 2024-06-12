#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace fsm::bumpgo
{

class BumpGoNode : public rclcpp::Node
{
public:
  BumpGoNode(std::string&& node_name = std::string("bump_go_pub"))
    : rclcpp::Node(std::move(node_name))
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("/twist", 10);
    subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/layse_scan",
        10,
        std::bind(&BumpGoNode::ReceiveLaserScan, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&BumpGoNode::ControlCircle, this));
  }

private:
  void ReceiveLaserScan(sensor_msgs::msg::LaserScan::UniquePtr msg);

  void ControlCircle();

  enum class RobotState
  {
    FORWARD,
    BACK,
    TURN,
    STOP,
  };

  RobotState state_{BumpGoNode::RobotState::STOP};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::UniquePtr last_;
};

} // namespace fsm::bumpgo