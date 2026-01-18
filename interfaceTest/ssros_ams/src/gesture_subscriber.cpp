#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/msg/head_touch_gesture.hpp"

void topic_callback(const test_interfaces::msg::HeadTouchGesture::SharedPtr msg)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "I heard: gesture=%d, value=%d, direction=%d", msg->gesture, msg->value, msg->direction);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gesture_subscriber");
  auto subscription = node->create_subscription<test_interfaces::msg::HeadTouchGesture>(
    "head_touch_gesture", 10, topic_callback);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}