//https://github.com/Ar-Ray-code/ps_ros2_common
#include "joy/ps_base.hpp"
#define JOY_VERSION PS4

#include <std_msgs/msg/string.hpp> // Corrected include

#if JOY_VERSION == PS4
#include "joy/ps4.hpp"
using namespace ps4;

#endif

namespace ps_ros2_common {

class example_joy : public rclcpp::Node, public ps
{
public:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_int; // Corrected message type
  std_msgs::msg::String joy_data_msg; // Declared message variable

  void sub_joy_thread(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    get_data(msg);

    // Create a message to store all the joystick data
    std::stringstream ss;
    
    ss << "square_btn:" << square_btn << std::endl;
    ss << "cross_btn :" << cross_btn << std::endl;
    ss << "circle_btn: " << circle_btn << std::endl;
    ss << "triangle_btn: " << triangle_btn << std::endl;
    ss << "L1_btn: " << L1_btn << std::endl;
    ss << "R1_btn: " << R1_btn << std::endl;
    ss << "L2_btn: " << L2_btn << std::endl;
    ss << "R2_btn: " << R2_btn << std::endl;
    ss << "up_btn: " << up_btn << std::endl;
    ss << "down_btn: " << down_btn << std::endl;
    ss << "left_btn: " << left_btn << std::endl;
    ss << "right_btn: " << right_btn << std::endl;
    ss << "select_btn: " << select_btn << std::endl;
    ss << "share_btn: " << share_btn << std::endl;
    ss << "create_btn: " << create_btn << std::endl;
    ss << "start_btn: " << start_btn << std::endl;
    ss << "options_btn: " << options_btn << std::endl;
    ss << "PS_btn: " << PS_btn << std::endl;
    ss << "joy_left_x: " << joy_left_x << std::endl;
    ss << "joy_left_y: " << joy_left_y << std::endl;
    ss << "joy_right_x: " << joy_right_x << std::endl;
    ss << "L2: " << L2 << std::endl;
    ss << "R2: " << R2 << std::endl;
    ss << "joy_right_y: " << joy_right_y << std::endl;
    ss << "d_pad_x: " << d_pad_x << std::endl;
    ss << "d_pad_y: " << d_pad_y << std::endl;

    joy_data_msg.data = ss.str();

    // Publish the message with all the joystick data
    pub_int->publish(joy_data_msg);
  }

  example_joy(const rclcpp::NodeOptions & options)
  : Node("joy_test", options)
  {
    using namespace std::chrono_literals;
    sub_joy =
      this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 1,
      std::bind(&ps_ros2_common::example_joy::sub_joy_thread, this, std::placeholders::_1)); // Corrected callback binding
    pub_int = this->create_publisher<std_msgs::msg::String>("output", 1); // Corrected message type
  }
};

} // namespace ps_ros2_common

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ps_ros2_common::example_joy)

int main(int argc, char ** argv)
{
  using namespace ps_ros2_common;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<example_joy>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
