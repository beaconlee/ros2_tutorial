#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class Subscriber : public rclcpp::Node
{
public:
  Subscriber(std::string&& name)
    : rclcpp::Node(name)
  {
    // 不知道该怎么创建一个 subscriber
    // 这样创一个 subscriber 如何确定订阅的 topic 的名字?
    // 这个名字是 topic 的名字
    subscriber_ = create_subscription<std_msgs::msg::String>(
        "string_test",
        100,
        std::bind(&Subscriber::Callback, this, std::placeholders::_1));
  }

private:
  // void Callback(std_msgs::msg::String::SharedPtr& msg)
  // error 这里传入一个引用时会报错？为什么？
  void Callback(std_msgs::msg::String::SharedPtr msg)
  {
    // Format string is not a string literal (potentially insecure) (fix available)clang(-Wformat-security)
    // 格式化字符串不是字符串文字（可能不安全）（修复程序可用）clang（-Wformat安全性）
    // RCLCPP_INFO(this->get_logger(), msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "Receive: %s", msg->data.c_str());
  }


  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  // rclcpp::TimerBase::SharedPtr timer_subscriber_;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto subscriber = std::make_shared<Subscriber>("subscriber");

  rclcpp::spin(subscriber);
  rclcpp::shutdown();
  return 0;
}