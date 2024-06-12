#include "bumpgo/bumpgo_node.h"

namespace fsm::bump_go
{

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


void BumpGoNode::Go2State(RobotState new_state)
{
  state_ = new_state;
  state_ts_ = now();
}
} // namespace fsm::bump_go