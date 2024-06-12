#include "bumpgo/bumpgo_node.hpp"
#include <utility>



namespace fsm::bump_go
{


BumpGoNode::BumpGoNode(std::string&& node_name)
  : rclcpp::Node(std::move(node_name))
{
  publisher_ = create_publisher<geometry_msgs::msg::Twist>("output_vel", 10);

  subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "input_scan",
      rclcpp::SensorDataQoS(),
      std::bind(&BumpGoNode::scan_callback, this, std::placeholders::_1));

  timer_ = create_wall_timer(50ms, std::bind(&BumpGoNode::control_cycle, this));
}

void BumpGoNode::scan_callback(
    sensor_msgs::msg::LaserScan::UniquePtr laser_scan)
{
  last_scan_ = std::move(laser_scan);
}

void BumpGoNode::control_cycle()
{
  if(!last_scan_)
  {
    return;
  }
  geometry_msgs::msg::Twist out_vel;

  // check_forward_2_stop();
  // switch(state_)
  // {
  //   case RobotState::FORWARD:
  //     twist.linear.set__x(0.5);
  //     break;
  //   case RobotState::BACK:
  //     twist.linear.set__x(-0.5);
  //     break;
  //   case RobotState::STOP:

  //     break;
  //   case RobotState::TURN:
  //     twist.angular.set__z(-0.1);
  //     break;
  //   default:
  //     break;
  // }

  switch(state_)
  {
    // tips 这种情况下如何设置 控制量呢？
    // 我的问题是 当前状态可能会改变
    case RobotState::FORWARD:
      out_vel.linear.set__x(SPEED_LINEAR);

      if(check_forward_2_stop())
      {
        go_state(RobotState::STOP);
      }

      if(check_forward_2_back())
      {
        go_state(RobotState::BACK);
      }
      break;

    case RobotState::BACK:
      out_vel.linear.set__x(-SPEED_LINEAR);

      if(check_back_2_turn())
      {
        go_state(RobotState::TURN);
      }

      break;

    case RobotState::TURN:
      out_vel.angular.set__z(SPEED_ANGULAR);
      if(check_turn_2_forward())
      {
        go_state(RobotState::FORWARD);
      }

      break;
    case RobotState::STOP:
      if(check_stop_2_forward())
      {
        go_state(RobotState::FORWARD);
      }
      break;
  }
  publisher_->publish(out_vel);
}

bool BumpGoNode::check_forward_2_stop()
{
  // auto elapsed = now() - last_scan_->header.stamp;
  // auto elapsed = now() - static_cast<rclcpp::Time>(last_scan_->header.stamp);
  // 这里不是类型转换，而是构造一个 rclcpp::Time 对象来进行计算
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed > SCAN_TIMEOUT;
  // rclcpp::Time 由 秒、纳秒 组成
  // param seconds part of the time in seconds since time epoch
  // param nanoseconds part of the time in nanoseconds since time epoch
  // rclcpp::Time now_ts = now();
  // if(((now_ts - state_ts_).seconds() > SCAN_TIMEOUT) &&
  //    (state_ != RobotState::STOP))
  // {
  //   go_state(RobotState::STOP);
  //   return;
  // }
  // if((state_ == RobotState::STOP) &&
  //    (now_ts - state_ts_).seconds() < SCAN_TIMEOUT)
  // {
  //   go_state(RobotState::FORWARD);
  //   return;
  // }
  // if((state_ == RobotState::BACK) &&
  //    ((now_ts - state_ts_).seconds() > kMaxBackTime))
  // {
  //   go_state(RobotState::TURN);
  //   return;
  // }
  // if((state_ == RobotState::TURN) &&
  //    ((now_ts - state_ts_).seconds() > kMaxBackTime))
  // {
  //   go_state(RobotState::FORWARD);
  //   return;
  // }
  // if((state_ == RobotState::FORWARD) && (last_scan_->ranges[333] > 2) &&
  //    (last_scan_->ranges[333] < 24))
  // {
  //   go_state(RobotState::BACK);
  //   return;
  // }
}

bool BumpGoNode::check_forward_2_back()
{
  auto pos = last_scan_->ranges.size() / 2;

  return last_scan_->ranges[pos] > OBSTACLE_DISTANCE;
}

bool BumpGoNode::check_back_2_turn()
{
  return (now() - state_ts_) > BACKING_TIME;
}

bool BumpGoNode::check_turn_2_forward()
{
  return (now() - state_ts_) > TURN_TIME;
}

bool BumpGoNode::check_stop_2_forward()
{
  auto elapsed = now() - rclcpp::Time(last_scan_->header.stamp);
  return elapsed < SCAN_TIMEOUT;
}


void BumpGoNode::go_state(RobotState new_state)
{
  state_ = new_state;
  state_ts_ = now();
}
} // namespace fsm::bump_go