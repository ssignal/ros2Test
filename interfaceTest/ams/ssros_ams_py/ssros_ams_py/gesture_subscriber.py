import rclpy
from rclpy.node import Node
from test_interfaces.msg import HeadTouchGesture


class GestureSubscriber(Node):
    def __init__(self):
        super().__init__("gesture_subscriber_py")
        self.subscription = self.create_subscription(
            HeadTouchGesture, "head_touch_gesture", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            'I heard: gesture="%d", value="%d", direction="%d"'
            % (msg.gesture, msg.value, msg.direction)
        )


def main(args=None):
    rclpy.init(args=args)
    gesture_subscriber = GestureSubscriber()
    rclpy.spin(gesture_subscriber)
    gesture_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
