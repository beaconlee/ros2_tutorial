#include "beacon/vff_avoidance/avoidance_node.hpp"
#include <rclcpp/executors.hpp>

using namespace std::chrono_literals;


namespace beacon
{
namespace planning
{
// 整体的设计思路是：
// 在进行扫描监测时，判断当前扫描的结果是否超时，超过一秒钟，机器人应该停止

// error 自己的问题是：不知道这部分该写在哪里
// tips 自己对于 scan_callback 和 control_cycle 的功能区分还不是很清楚

VFFAvoidanceNode::VFFAvoidanceNode()
  : rclcpp::Node("vff_avoidance")
{
  vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 100);
  vff_debug_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("vff_debug", 100);

  laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "input_scan",
      rclcpp::SensorDataQoS(),
      std::bind(&VFFAvoidanceNode::scan_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(50ms,
                             std::bind(&VFFAvoidanceNode::control_cycle, this));
}


void VFFAvoidanceNode::scan_callback(sensor_msgs::msg::LaserScan::UniquePtr msg)
{
  // 这里的一个问题是如何对 unique_ptr 进行赋值，并且提高效率
  // 直接使用 移动语义
  last_scan_ = std::move(msg);
}


void VFFAvoidanceNode::control_cycle()
{
  if(laser_sub_ == nullptr || (now() - last_scan_->header.stamp) > 1s)
  {
    return;
  }

  // VFFVectors vff_vec = get_vff(*last_scan_);
  // 这里可以使用 const 引用来提高效率
  const VFFVectors& vff_vec = get_vff(*last_scan_);

  // std::vector<float> result = vff_vec.result;
  // 同样这里可以使用常量引用
  // 有个问题是：如果使用普通引用，就会报错，但是使用 const& 就不会报错
  const std::vector<float>& result = vff_vec.result;

  geometry_msgs::msg::Twist vel;

  // error 第一个问题，如何给 twist 赋值
  // error 错误案例
  // float x = result[0];
  // twist.set__linear(x);


  // float angle = atan(result[0] / result[1]);
  // float angle = std::atan2(result[1], result[0]);
  double angle = std::atan2(result[1], result[0]);
  // float speed = std::sqrt(result[0] * result[0] + result[1] * result[1]);
  // float module = std::sqrt(result[0] * result[0] + result[1] * result[1]);
  double module = std::sqrt(result[0] * result[0] + result[1] * result[1]);

  // 关于 Twist 的 linear 和 angular 的 x、y、z 已经忘掉啦
  vel.linear.x = std::clamp(module, 0.0, 0.3);
  vel.angular.z = std::clamp(angle, -0.5, 0.5);

  vel_pub_->publish(vel);

  // 关于 debug 的输出自己也不会
  if(vff_debug_pub_->get_subscription_count() > 0)
  {
    vff_debug_pub_->publish(get_debug_vff(vff_vec));
  }
}


VFFVectors VFFAvoidanceNode::get_vff(const sensor_msgs::msg::LaserScan& scan)
{
  static const float OBSTACLE_DISTANCE = 1.;


  VFFVectors vff_vec;
  vff_vec.attractive = {OBSTACLE_DISTANCE, 0.};

  // if(dist_min < OBSTACLE_DISTANCE)
  // {
  //   float angle = scan.angle_min + scan.angle_increment * idx_min;
  //   float angle_orthogonal = angle + M_PI;
  //   float complementary_dist = OBSTACLE_DISTANCE - dist_min;
  //   vff_vec.repulsive = {complementary_dist * cos(angle_orthogonal),
  //                        complementary_dist * sin(angle_orthogonal)};
  // }

  vff_vec.repulsive = {0., 0.};
  // vff_vec.result = vff_vec.attractive;
  // 初始化 result 假设它周围没有障碍物
  vff_vec.result = {1.0, 0.};

  int idx_min = std::min_element(scan.ranges.begin(), scan.ranges.end()) -
                scan.ranges.begin();
  float dist_min = scan.ranges[idx_min];

  if(dist_min < OBSTACLE_DISTANCE)
  {
    float angle = scan.angle_min + scan.angle_increment * idx_min;

    // 这里加上一个 PI，是说明机器人要往远离障碍物的方向移动
    // 因此加上一个 PI，去反方向
    float angle_orthogonal = angle + M_PI;

    float complementary_dist = OBSTACLE_DISTANCE - dist_min;

    // 这里为什么是 cos 和 sin？需要仔细想一下
    vff_vec.repulsive[0] = complementary_dist * cos(angle_orthogonal);
    vff_vec.repulsive[1] = complementary_dist * sin(angle_orthogonal);

    // vff_vec.result[0] = vff_vec.attractive[0] + vff_vec.repulsive[0];
    // vff_vec.result[1] = vff_vec.attractive[1] + vff_vec.repulsive[1];
  }

  // tips 自己的想法是直接将 result 初始化为 attractive，如果有 repulsive，再对 result 进行操作
  vff_vec.result[0] = vff_vec.attractive[0] + vff_vec.repulsive[0];
  vff_vec.result[1] = vff_vec.attractive[1] + vff_vec.repulsive[1];

  return vff_vec;
}


visualization_msgs::msg::MarkerArray
VFFAvoidanceNode::get_debug_vff(const VFFVectors& vff_vector)
{
  visualization_msgs::msg::MarkerArray marker_arr;
  marker_arr.markers.push_back(
      make_maker(vff_vector.attractive, VFFColor::BLUE));
  marker_arr.markers.push_back(make_maker(vff_vector.repulsive, VFFColor::RED));
  marker_arr.markers.push_back(make_maker(vff_vector.result, VFFColor::GREEN));

  return marker_arr;
}


visualization_msgs::msg::Marker
VFFAvoidanceNode::make_maker(const std::vector<float>& vector, VFFColor color)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = now();
  marker.header.frame_id = "base_footprint";

  // 没有设置 marker 的 type 和 id
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.id = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point end;

  // start.set__x(0.0);
  // start.set__y(0.0);
  start.x = 0.;
  start.y = 0.;

  // end.set__x(vector[0]);
  // end.set__y(vector[1]);
  end.x = vector[0];
  end.y = vector[1];

  marker.points = {start, end};

  // 这里对颜色赋值自己忘记要怎么弄了
  switch(color)
  {
    // 注意这里设置 rgb 值是 0 到 1. 不是到 255
    case VFFColor::RED:
      marker.id = 0;
      marker.color.r = 1.0;
      break;
    case VFFColor::GREEN:
      marker.id = 1;
      marker.color.g = 1.0;
      break;
    case VFFColor::BLUE:
      marker.id = 2;
      marker.color.b = 1.0;
      break;
    case VFFColor::NUM_COLOR:
      marker.id = 3;
      marker.color.r = 1.;
      marker.color.g = 1.;
      marker.color.b = 1.;
      break;
  }

  marker.color.a = 1.0;

  return marker;
}


} // namespace planning
} // namespace beacon

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nodes = beacon::planning::VFFAvoidanceNode::make_shared("avoidance");


  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(nodes);

  executor->spin();

  rclcpp::shutdown();
  return 0;
}