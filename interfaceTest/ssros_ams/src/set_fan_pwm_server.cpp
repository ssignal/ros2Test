#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/srv/set_fan_pwm.hpp"

#include <memory>

void set_fan_pwm(const std::shared_ptr<test_interfaces::srv::SetFanPWM::Request> request,
          std::shared_ptr<test_interfaces::srv::SetFanPWM::Response>      response)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\npwm: %d", request->pwm);
  response->success = true;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_fan_pwm_server");

  rclcpp::Service<test_interfaces::srv::SetFanPWM>::SharedPtr service =
    node->create_service<test_interfaces::srv::SetFanPWM>("set_fan_pwm", &set_fan_pwm);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to set fan pwm.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
