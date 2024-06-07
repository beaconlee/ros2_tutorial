#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode(std::string&& name)
    : rclcpp::Node(std::move(name))
  {
    publisher_ = create_publisher<std_msgs::msg::String>("publish", 100);
    // publisher_timer_ = rclcpp::create_timer(std::bind(PublisherMsg, 10));
    publisher_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&PublisherNode::PublisherMsg, this));
  }

private:
  void PublisherMsg()
  {
    std_msgs::msg::String msg = std_msgs::msg::String();
    msg.data = "天下无敌" + std::to_string(count_++);
    publisher_->publish(msg);
  }


  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publisher_timer_;
  uint64_t count_{0};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Call to non-static member function without an object argumentclang(member_call_without_object)
  // 调用一个非静态函数但是没有一个对象
  // auto publisher =
  // rclcpp::Node::create_publisher<std_msgs::msg::String>("publisher", 10);

  // PublisherNode node("first_publisher");
  auto node = std::make_shared<PublisherNode>("first_publisher");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}