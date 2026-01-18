#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/msg/head_touch_gesture.hpp"
#include "builtin_interfaces/msg/time.hpp"

using namespace std::chrono_literals;

class GesturePublisher : public rclcpp::Node
{
public:
  GesturePublisher()
  : Node("gesture_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<test_interfaces::msg::HeadTouchGesture>("head_touch_gesture", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&GesturePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = test_interfaces::msg::HeadTouchGesture();
    message.stamp = this->get_clock()->now();
    message.gesture = count_ % 7;
    message.value = count_ % 2;
    message.direction = count_ % 3;
    
    RCLCPP_INFO(this->get_logger(), "Publishing: gesture=%d, value=%d, direction=%d", message.gesture, message.value, message.direction);
    publisher_->publish(message);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<test_interfaces::msg::HeadTouchGesture>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GesturePublisher>());
  rclcpp::shutdown();
  return 0;
}