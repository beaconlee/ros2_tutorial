#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace fsm::bump_go
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
                               std::bind(&BumpGoNode::ControlCycle, this));
  }

private:
  enum class RobotState
  {
    FORWARD,
    BACK,
    TURN,
    STOP,
  };

  void ReceiveLaserScan(sensor_msgs::msg::LaserScan::UniquePtr laser_scan);

  void ControlCycle();

  void Go2State(RobotState new_state);


  RobotState state_{BumpGoNode::RobotState::STOP};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time state_ts_; // ts is timestamp 时间戳
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

} // namespace fsm::bump_go