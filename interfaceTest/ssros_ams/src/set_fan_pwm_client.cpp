#include "rclcpp/rclcpp.hpp"
#include "test_interfaces/srv/set_fan_pwm.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: set_fan_pwm_client PWM");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("set_fan_pwm_client");
  rclcpp::Client<test_interfaces::srv::SetFanPWM>::SharedPtr client =
    node->create_client<test_interfaces::srv::SetFanPWM>("set_fan_pwm");

  auto request = std::make_shared<test_interfaces::srv::SetFanPWM::Request>();
  request->pwm = atoll(argv[1]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success: %d", result.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service set_fan_pwm");
  }

  rclcpp::shutdown();
  return 0;
}