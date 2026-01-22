#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "test_interfaces/action/spin.hpp"

#include <memory>
#include <thread>

class SpinActionServer : public rclcpp::Node
{
public:
  using Spin = test_interfaces::action::Spin;
  using GoalHandleSpin = rclcpp_action::ServerGoalHandle<Spin>;

  explicit SpinActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("spin_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Spin>(
      this,
      "spin",
      std::bind(&SpinActionServer::handle_goal, this, _1, _2),
      std::bind(&SpinActionServer::handle_cancel, this, _1),
      std::bind(&SpinActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Spin>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Spin::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with target yaw %f", goal->target_yaw);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSpin> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleSpin> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&SpinActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleSpin> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Spin::Feedback>();
    auto & angular_distance_traveled = feedback->angular_distance_traveled;
    angular_distance_traveled = 0;
    auto result = std::make_shared<Spin::Result>();

    for (int i = 0; (i < goal->target_yaw) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->error_code = Spin::Result::UNKNOWN;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
      }
      // Update sequence
      angular_distance_traveled = i;
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->error_code = Spin::Result::NONE;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    }
  }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_server = std::make_shared<SpinActionServer>();
    rclcpp::spin(action_server);
    rclcpp::shutdown();
    return 0;
}