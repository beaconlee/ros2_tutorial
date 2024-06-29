#include <gtest/gtest.h>
#include <limits>
#include <vector>
#include <memory>

#include "sensor_msgs/msg/laser_scan.hpp"
#include "beacon/vff_avoidance/avoidance_node.hpp"


class AvoidanceNodeTest : public beacon::planning::VFFAvoidanceNode
{
public:
  using VFFVectors = beacon::planning::VFFVectors;

  VFFVectors get_vff_test(const sensor_msgs::msg::LaserScan& scan)
  {
    return get_vff(scan);
  }

  visualization_msgs::msg::MarkerArray get_debug_vff_test(const VFFVectors& vff_vectors)
  {
    return get_debug_vff(vff_vectors);
  }
};

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

TEST(vff_tests, get_vff)
{
  auto node_avoidance = AvoidanceNodeTest();
  rclcpp::Time ts = node_avoidance.now();
  
  auto res1 = node_avoidance.get_vff_test(get_scan_test_1(ts));
  ASSERT_EQ(res1.attractive, std::vector<float>({1.0f, 0.0f}));
  ASSERT_EQ(res1.repulsive, std::vector<float>({0.0f, 0.0f}));
  ASSERT_EQ(res1.result, std::vector<float>({1.0f, 0.0f}));
}


int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}