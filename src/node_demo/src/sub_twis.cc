#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class TwistController : public rclcpp::Node
{
public:
  TwistController()
    : Node("twist_controller")
  {
    // Create a subscriber for the Twist message
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", // Topic name
        rclcpp::QoS(10),
        std::bind(&TwistController::twist_callback,
                  this,
                  std::placeholders::_1));

    // Create a publisher for the joint states
    publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                             rclcpp::QoS(10));

    // Declare parameters
    this->declare_parameter("linear_scaling_factor", 1.0);
    this->declare_parameter("angular_scaling_factor", 1.0);
  }

private:
  void
  twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Read parameters
    double linear_scaling =
        this->get_parameter("linear_scaling_factor").as_double();
    double angular_scaling =
        this->get_parameter("angular_scaling_factor").as_double();

    // Convert Twist message to joint commands
    double scaled_linear_x = msg->linear.x * linear_scaling;
    double scaled_angular_z = msg->angular.z * angular_scaling;

    static double value = 1.0;
    value += 10;


    // Create and publish JointState message
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->get_clock()->now();
    // Fill in joint names and values according to your robot model
    joint_state_msg.name = {"base_2_wheel_left_front",
                            "base_2_wheel_right_front",
                            "base_2_wheel_right_back",
                            "base_2_wheel_left_back",
                            "base_2_arm_base",
                            "arm_button_2_arm_base",
                            "arm_upper_to_arm_button",
                            "right_gripper_2_arm_upper",
                            "left_gripper_2_arm_upper"};
    joint_state_msg.position = {value,
                                value,
                                value,
                                value,
                                value,
                                value,
                                value,
                                value,
                                value};            // Example positions
    joint_state_msg.velocity = {2, 2, 2, 2};       // Example velocities
    joint_state_msg.effort = {0.5, 0.5, 0.5, 0.5}; // Example efforts

    publisher_->publish(joint_state_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};

int
main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistController>());
  rclcpp::shutdown();
  return 0;
}
