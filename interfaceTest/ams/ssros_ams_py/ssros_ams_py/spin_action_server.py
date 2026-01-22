import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from test_interfaces.action import Spin


class SpinActionServer(Node):
    def __init__(self):
        super().__init__("spin_action_server_py")
        self._action_server = ActionServer(
            self,
            Spin,
            "spin",
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(
            "Received goal request with target yaw %f" % goal_request.target_yaw
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received request to cancel goal")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")
        feedback_msg = Spin.Feedback()
        feedback_msg.angular_distance_traveled = 0.0

        for i in range(int(goal_handle.request.target_yaw)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal Canceled")
                return Spin.Result()

            feedback_msg.angular_distance_traveled = float(i)
            self.get_logger().info("Publish Feedback")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        self.get_logger().info("Goal Succeeded")

        result = Spin.Result()
        result.error_code = Spin.Result.NONE
        return result


def main(args=None):
    rclpy.init(args=args)
    spin_action_server = SpinActionServer()
    rclpy.spin(spin_action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
