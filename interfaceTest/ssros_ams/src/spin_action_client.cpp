#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_interfaces/action/spin.hpp"

#include <memory>
#include <string>
#include <iostream>

class SpinActionClient : public rclcpp::Node
{
public:
  using Spin = test_interfaces::action::Spin;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

  explicit SpinActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("spin_action_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Spin>(
      this,
      "spin");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&SpinActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Spin::Goal();
    goal_msg.target_yaw = 10.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Spin>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SpinActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&SpinActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&SpinActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Spin>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleSpin::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleSpin::SharedPtr,
    const std::shared_ptr<const Spin::Feedback> feedback)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Feedback received: %f",
      feedback->angular_distance_traveled);
  }

  void result_callback(const GoalHandleSpin::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Result received: %d", result.result->error_code);
    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<SpinActionClient>();
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}