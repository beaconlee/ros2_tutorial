#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>



namespace beacon
{
namespace planning
{
struct VFFVectors
{
  std::vector<float> attractive;
  std::vector<float> repulsive;
  std::vector<float> result;
};

enum class VFFColor
{
  RED,
  GREEN,
  BLUE,
  NUM_COLOR
};


class VFFAvoidanceNode : public rclcpp::Node
{
public:
  VFFAvoidanceNode();
  void scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg);
  void control_cycle();

private:
  VFFVectors get_vff(sensor_msgs::msg::LaserScan& scan);

  visualization_msgs::msg::MarkerArray
  get_vff_debug(const VFFVectors& vff_vector);

  visualization_msgs::msg::Marker make_maker(const std::vector<float>& vector,
                                             VFFColor color);

private:
  // 自己对于 twist 这个 message 还不是很熟悉，不知道它的具体功能是什么
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      vff_debug_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::UniquePtr last_scan_;
};


} // namespace planning
} // namespace beacon