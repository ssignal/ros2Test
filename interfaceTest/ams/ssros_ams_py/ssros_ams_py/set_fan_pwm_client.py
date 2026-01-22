import sys
from test_interfaces.srv import SetFanPWM
import rclpy
from rclpy.node import Node


class SetFanPWMClient(Node):
    def __init__(self):
        super().__init__("set_fan_pwm_client_py")
        self.cli = self.create_client(SetFanPWM, "set_fan_pwm")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = SetFanPWM.Request()

    def send_request(self):
        self.req.pwm = int(sys.argv[1])
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print("usage: set_fan_pwm_client PWM")
        sys.exit(1)

    set_fan_pwm_client = SetFanPWMClient()
    set_fan_pwm_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(set_fan_pwm_client)
        if set_fan_pwm_client.future.done():
            try:
                response = set_fan_pwm_client.future.result()
            except Exception as e:
                set_fan_pwm_client.get_logger().info("Service call failed %r" % (e,))
            else:
                set_fan_pwm_client.get_logger().info(
                    "Result of set_fan_pwm: for %d, success: %d"
                    % (set_fan_pwm_client.req.pwm, response.success)
                )
            break

    set_fan_pwm_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
