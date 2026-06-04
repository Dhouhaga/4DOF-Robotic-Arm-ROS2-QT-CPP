#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class CommandRouter : public rclcpp::Node
{
public:
  CommandRouter()
  : Node("command_router")
  {
    user_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10);
    viz_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/arm_viz/commands", 10);

    user_command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/arm_controller/commands", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        user_command_pub_->publish(*msg);
        viz_command_pub_->publish(*msg);
      });

    RCLCPP_INFO(this->get_logger(), "command_router ready - listening on /arm_controller/commands");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr user_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr viz_command_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr user_command_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandRouter>());
  rclcpp::shutdown();
  return 0;
}
