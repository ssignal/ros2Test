from test_interfaces.srv import SetFanPWM

import rclpy
from rclpy.node import Node

class SetFanPWMServer(Node):

    def __init__(self):
        super().__init__('set_fan_pwm_server')
        self.srv = self.create_service(SetFanPWM, 'set_fan_pwm', self.set_fan_pwm_callback)

    def set_fan_pwm_callback(self, request, response):
        self.get_logger().info('Incoming request\npwm: %d' % (request.pwm))
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    set_fan_pwm_server = SetFanPWMServer()
    rclpy.spin(set_fan_pwm_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
