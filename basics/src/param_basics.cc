#include <rclcpp/rclcpp.hpp>

// 关于参数的实现，自己还是忘记啦，再看一遍


class ParamNode : public rclcpp::Node
{
public:
  ParamNode(std::string&& node_name)
    : rclcpp::Node(std::move(node_name))
  {
    declare_parameter("num_of_params", 0);
    // declare_parameter("topics_name", "");
    // declare_parameter("topics_type", "");
    declare_parameter("topics_name", std::vector<std::string>());
    declare_parameter("topics_type", std::vector<std::string>());

    get_parameter("num_of_params", num_of_params_);
    get_parameter("topics_name", topics_name_);
    get_parameter("topics_type", topics_type_);

    if(num_of_params_ < 0)
    {
      RCLCPP_ERROR(get_logger(), "params < 0");
    }

    if(topics_type_.size() != topics_name_.size())
    {
      RCLCPP_ERROR(get_logger(), "get topics info error");
    }


    RCLCPP_INFO_STREAM(get_logger(), "num_of_params:" << num_of_params_);
    // 这里自己不知道该如何去实现输出一个 vector
    // RCLCPP_INFO_STREAM(get_logger(), "topics_name:" << topics_name_);
    // RCLCPP_INFO_STREAM(get_logger(), "topics_type:" << num_of_params_);
    for(uint idx = 0; idx < topics_name_.size(); ++idx)
    {
      RCLCPP_INFO_STREAM(get_logger(),
                         "topics_type:" << topics_type_[idx]
                                        << "   topics_name: "
                                        << topics_name_[idx]);
    }
  }

private:
  uint num_of_params_;
  std::vector<std::string> topics_name_;
  std::vector<std::string> topics_type_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto param_node = std::make_shared<ParamNode>("param_node");

  rclcpp::spin(param_node);
  rclcpp::shutdown();

  return 0;
}