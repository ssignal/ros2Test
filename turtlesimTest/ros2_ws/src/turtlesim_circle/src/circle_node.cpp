#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class CircleNode : public rclcpp::Node
{
public:
  CircleNode()
  : Node("circle_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    timer_ = this->create_wall_timer(
      10ms, std::bind(&CircleNode::publish_velocity, this));
  }

private:
  void publish_velocity()
  {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 2.0;
    message.angular.z = 1.0;
    publisher_->publish(message);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleNode>());
  rclcpp::shutdown();
  return 0;
}