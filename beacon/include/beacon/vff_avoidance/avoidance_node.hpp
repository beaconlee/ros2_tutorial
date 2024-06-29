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

  // 定义为 protected 的方法，方便使用 gtest 进行继承测试
protected:
  // VFFVectors get_vff(sensor_msgs::msg::LaserScan& scan);
  // tips 这里应该是 const &
  VFFVectors get_vff(const sensor_msgs::msg::LaserScan& scan);

  visualization_msgs::msg::MarkerArray
  get_debug_vff(const VFFVectors& vff_vector);

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