import rclpy
from rclpy.node import Node
from test_interfaces.msg import HeadTouchGesture
from builtin_interfaces.msg import Time


class GesturePublisher(Node):
    def __init__(self):
        super().__init__("gesture_publisher_py")
        self.publisher_ = self.create_publisher(
            HeadTouchGesture, "head_touch_gesture", 10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count_ = 0

    def timer_callback(self):
        msg = HeadTouchGesture()
        msg.stamp = self.get_clock().now().to_msg()
        msg.gesture = self.count_ % 7
        msg.value = self.count_ % 2
        msg.direction = self.count_ % 3

        self.get_logger().info(
            "Publishing: gesture=%d, value=%d, direction=%d"
            % (msg.gesture, msg.value, msg.direction)
        )
        self.publisher_.publish(msg)
        self.count_ += 1


def main(args=None):
    rclpy.init(args=args)
    gesture_publisher = GesturePublisher()
    rclpy.spin(gesture_publisher)
    gesture_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
