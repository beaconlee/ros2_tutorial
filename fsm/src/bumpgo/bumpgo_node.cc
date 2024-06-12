#include "bumpgo/bumpgo_node.h"

namespace fsm::bump_go
{

constexpr double kMaxInvaild = 1.;
constexpr double kMaxBackTime = 2.;

void BumpGoNode::ReceiveLaserScan(
    sensor_msgs::msg::LaserScan::UniquePtr laser_scan)
{
  last_scan_ = std::move(laser_scan);
}

void BumpGoNode::ControlCycle()
{
  if(!last_scan_)
  {
    return;
  }

  CheckState();

  geometry_msgs::msg::Twist twist;

  switch(state_)
  {
    case RobotState::FORWARD:
      twist.linear.set__x(0.5);
      break;
    case RobotState::BACK:
      twist.linear.set__x(-0.5);
      break;
    case RobotState::STOP:

      break;
    case RobotState::TURN:
      twist.angular.set__z(-0.1);
      break;
    default:
      break;
  }

  publisher_->publish(twist);
}

void BumpGoNode::CheckState()
{
  rclcpp::Time now_ts = now();

  if(((now_ts - state_ts_).seconds() > kMaxInvaild) &&
     (state_ != RobotState::STOP))
  {
    Go2State(RobotState::STOP);
    return;
  }

  if((state_ == RobotState::STOP) &&
     (now_ts - state_ts_).seconds() < kMaxInvaild)
  {
    Go2State(RobotState::FORWARD);
    return;
  }

  if((state_ == RobotState::BACK) &&
     ((now_ts - state_ts_).seconds() > kMaxBackTime))
  {
    Go2State(RobotState::TURN);
    return;
  }

  if((state_ == RobotState::TURN) &&
     ((now_ts - state_ts_).seconds() > kMaxBackTime))
  {
    Go2State(RobotState::FORWARD);
    return;
  }

  if((state_ == RobotState::FORWARD) && (last_scan_->ranges[333] > 2) &&
     (last_scan_->ranges[333] < 24))
  {
    Go2State(RobotState::BACK);
    return;
  }
}



void BumpGoNode::Go2State(RobotState new_state)
{
  state_ = new_state;
  state_ts_ = now();
}
} // namespace fsm::bump_go