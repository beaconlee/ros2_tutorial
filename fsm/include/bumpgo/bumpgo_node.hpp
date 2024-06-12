#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>


namespace fsm::bump_go
{
// ms s 字面量是 C++14 中的 std::chrono 的一部分。
// 可以确保使用 ms 等字面常量
using namespace std::chrono_literals;

class BumpGoNode : public rclcpp::Node
{
public:
  explicit BumpGoNode(std::string&& node_name = std::string("bump_go"));

private:
  enum class RobotState
  {
    FORWARD,
    BACK,
    TURN,
    STOP,
  };

  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr laser_scan);

  void control_cycle();

  void go_state(RobotState new_state);

  bool check_forward_2_stop();
  bool check_forward_2_back();
  bool check_back_2_turn();
  bool check_turn_2_forward();
  bool check_stop_2_forward();

  const rclcpp::Duration TURN_TIME{2s};
  const rclcpp::Duration BACKING_TIME{2s};
  const rclcpp::Duration SCAN_TIMEOUT{1s};

  static constexpr float SPEED_LINEAR{0.3f};
  static constexpr float SPEED_ANGULAR{0.3f};
  static constexpr float OBSTACLE_DISTANCE{1.0f};

  RobotState state_{BumpGoNode::RobotState::STOP};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time state_ts_; // ts is timestamp 时间戳
  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};

} // namespace fsm::bump_go