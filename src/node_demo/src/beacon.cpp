#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;


class PublishNode : public rclcpp::Node
{
public:
  PublishNode()
    : rclcpp::Node("publisher")
  {
    publisher_ =
        create_publisher<std_msgs::msg::String>("beacon_executors", 100);
    pub_timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                   std::bind(&PublishNode::publiser, this));
  }

private:
  void
  publiser()
  {
    std_msgs::msg::String str;
    str.data = "publisher";
    RCLCPP_INFO(get_logger(), "this is pub");

    // 这里如何发布消息呢？
    publisher_->publish(str);
  }
  // error 这里用错类型啦
  // rclcpp::PublisherBase::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

class SubNode : public rclcpp::Node
{
public:
  SubNode()
    : rclcpp::Node("subscriver")
  {
    // well
    // 这里自己发现写的不对的时候，自己去查看了当前函数的声明，然后该了过来，没有再去看之前写的代码是如何写的了
    subscription_ = create_subscription<std_msgs::msg::String>(
        "beacon_executors",
        100,
        std::bind(&SubNode::Sub, this, std::placeholders::_1));
  }

private:
  void
  Sub(std_msgs::msg::String::SharedPtr stptr)
  {
    // str.data 是一个 string 类型
    // 将 string 类型转化为 char* 类型
    // std::string data = stptr->data;
    // RCLCPP_INFO 最终会调用 rcutils_log 函数，这个函数期望接收一个 "const char*" 类型的字符串
    // 然后使用格式化字符串"%s"来传递这个转换后的字符串。
    RCLCPP_INFO(get_logger(), "%s, %s", "this is sub", stptr->data.c_str());
  }
  // rclcpp::SubscriptionBase::SharedPtr subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};


class JointStatePublisher : public rclcpp::Node
{
public:
  JointStatePublisher()
    : Node("beacon_joint_publisher")
  {
    // 创建一个发布器，发布到 /joint_states 话题
    publisher_ =
        this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
                                                             10);

    // 定时器每秒发布一次关节状态
    timer_ = this->create_wall_timer(1s, [this]() { publish_joint_state(); });

    RCLCPP_INFO(this->get_logger(),
                "Joint State Publisher Node has been started.");
  }


private:
  void
  publish_joint_state()
  {
    auto message = sensor_msgs::msg::JointState();
    message.name = {"base_2_arm_base",
                    "arm_button_2_arm_base",
                    "arm_upper_to_arm_button",
                    "right_gripper_2_arm_upper",
                    "left_gripper_2_arm_upper"};
    static double value = 0.0;
    value += 10;
    message.header.stamp = this->get_clock()->now();


    message.position = {value, value, value, value, value};

    value += 0.01;
    message.velocity = {0.1, 0.2};

    message.effort = {0.0, 0.0};

    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Joint State publish message.");
  }

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int
main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  // auto pub = rclcpp::Node::make_shared("pub");
  // 这里怎么使用 rclcpp 的语法呢？直接传入 PubNode 的类型

  auto pub = std::make_shared<PublishNode>();
  // auto sub = std::make_shared<SubNode>();
  auto joint = std::make_shared<JointStatePublisher>();

  // auto executors = rclcpp::Executor<rclcpp::SignalHandlerOptions>();
  // 这里不知道该如何实现 executors 了
  // auto executors = std::make_shared<rclcpp::Executor>();

  // error 这里自己犯了一个很大的错，Executor 是类名，而不是命名空间，自己之前再写的时候完全没有分清
  auto executors =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executors->add_node(pub);
  // executors->add_node(sub);
  // executors->add_node(joint);

  executors->spin();

  rclcpp::shutdown();

  return 0;
}