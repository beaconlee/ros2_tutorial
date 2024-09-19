// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2_ros/transform_broadcaster.h>


// class OdomPublisher : public rclcpp::Node
// {
// public:
//   OdomPublisher()
//     : Node("odom_publisher")
//   {
//     // 创建发布器，用于发布里程计信息
//     odom_publisher_ =
//         this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

//     // 创建 TF 广播器
//     tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

//     // 设置定时器以固定频率发布数据
//     timer_ =
//         this->create_wall_timer(100ms,
//                                 std::bind(&OdomPublisher::publish_odom, this));
//   }

// private:
//   void
//   publish_odom()
//   {
//     // 创建并填充里程计消息
//     nav_msgs::msg::Odometry odom_msg;
//     odom_msg.header.stamp = this->get_clock()->now();
//     odom_msg.header.frame_id = "odom";
//     odom_msg.child_frame_id = "base_link";

//     // 示例：设定位置信息
//     odom_msg.pose.pose.position.x = 0.0;
//     odom_msg.pose.pose.position.y = 0.0;
//     odom_msg.pose.pose.position.z = 0.0;

//     // 示例：设定四元数表示的姿态信息（这里为单位四元数）
//     odom_msg.pose.pose.orientation.x = 0.0;
//     odom_msg.pose.pose.orientation.y = 0.0;
//     odom_msg.pose.pose.orientation.z = 0.0;
//     odom_msg.pose.pose.orientation.w = 1.0;

//     // 示例：设定线速度和角速度
//     odom_msg.twist.twist.linear.x = 0.1;  // 每秒前进 0.1 米
//     odom_msg.twist.twist.angular.z = 0.1; // 每秒旋转 0.1 弧度

//     // 发布里程计消息
//     odom_publisher_->publish(odom_msg);

//     // 创建并发布 TF 变换
//     geometry_msgs::msg::TransformStamped transform_stamped;
//     transform_stamped.header.stamp = this->get_clock()->now();
//     transform_stamped.header.frame_id = "odom";
//     transform_stamped.child_frame_id = "base_link";

//     transform_stamped.transform.translation.x = 0.0;
//     transform_stamped.transform.translation.y = 0.0;
//     transform_stamped.transform.translation.z = 0.0;
//     transform_stamped.transform.rotation = odom_msg.pose.pose.orientation;

//     // 广播 TF 变换
//     tf_broadcaster_->sendTransform(transform_stamped);
//   }

//   // 发布器和广播器
//   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
//   std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
//   rclcpp::TimerBase::SharedPtr timer_;
// };

// int
// main(int argc, char* argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<OdomPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
    : Node("odom_publisher")
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0)
    , linear_speed_(0.1)
    , angular_speed_(0.1)
  {
    // 创建发布者
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    // Transform 广播器，用于发布 tf 数据
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // 定时器，用于定期发布 odom 数据
    timer_ =
        this->create_wall_timer(100ms,
                                std::bind(&OdomPublisher::publish_odom, this));

    last_time_ = this->get_clock()->now();
  }

private:
  void
  publish_odom()
  {
    // 当前时间
    rclcpp::Time current_time = this->get_clock()->now();

    // 计算时间间隔
    double dt = (current_time - last_time_).seconds();

    // 更新机器人的位姿（x, y, theta）
    double delta_x = linear_speed_ * std::cos(theta_) * dt;
    double delta_y = linear_speed_ * std::sin(theta_) * dt;
    double delta_theta = angular_speed_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // 发布 odom 数据
    auto odom_msg = nav_msgs::msg::Odometry();

    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "odom";

    // 设置机器人的位置
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    // 设置机器人的朝向（四元数表示）
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 设置机器人的线速度和角速度
    odom_msg.twist.twist.linear.x = linear_speed_;
    odom_msg.twist.twist.angular.z = angular_speed_;

    // 发布 odom 消息
    odom_pub_->publish(odom_msg);

    // 发布 tf 变换
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";

    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = q.x();
    odom_tf.transform.rotation.y = q.y();
    odom_tf.transform.rotation.z = q.z();
    odom_tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(odom_tf);

    last_time_ = current_time;
  }

  // 发布者、定时器和 tf 广播器
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 机器人的状态
  double x_, y_, theta_;
  double linear_speed_, angular_speed_;
  rclcpp::Time last_time_;
};

int
main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
