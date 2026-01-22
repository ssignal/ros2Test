#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using namespace std::chrono_literals;

enum class State {
    FORWARD,
    TURN
};

class TriangleNode : public rclcpp::Node
{
public:
  TriangleNode()
  : Node("triangle_node"), state_(State::FORWARD), forward_distance_(0.0), angle_distance_(0.0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      100ms, std::bind(&TriangleNode::move_triangle, this));
  }

private:
  void move_triangle()
  {
    auto message = geometry_msgs::msg::Twist();

    switch (state_)
    {
      case State::FORWARD:
        message.linear.x = 2.0;
        forward_distance_ += 0.2;
        if (forward_distance_ >= 4.0)
        {
            forward_distance_ = 0.0;
            state_ = State::TURN;
        }
        break;

      case State::TURN:
        message.angular.z = 2.0 * M_PI / 3.0;
        angle_distance_ += 2.0*M_PI/30.0;
        if (angle_distance_ >= 2.0 * M_PI / 3.0)
        {
            angle_distance_ = 0.0;
            state_ = State::FORWARD;
        }
        break;
    }
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  State state_;
  double forward_distance_;
  double angle_distance_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TriangleNode>());
  rclcpp::shutdown();
  return 0;
}