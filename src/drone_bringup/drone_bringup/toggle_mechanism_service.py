import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64


class ToggleMechanismService(Node):
    def __init__(self):
        super().__init__('toggle_mechanism_service')
        self.srv = self.create_service(SetBool, 'toggle_mechanism', self.callback_toggle)

        # Publisher to command the prismatic joint (slider height)
        self.slider_pub = self.create_publisher(Float64, '/column_to_slider/effort_command', 10)

        # Slider effort to move up/down; values depend on joint direction
        self.closing_effort = 50.0   # Upward effort
        self.opening_effort = -30.0  # Downward effort
        self.duration_sec = 1.5      # How long to apply the effort

        self.get_logger().info("Service 'toggle_mechanism' ready.")

    def callback_toggle(self, request, response):
        # True = close; False = open
        effort = self.closing_effort if request.data else self.opening_effort
        direction = 'closing' if request.data else 'opening'
        self.get_logger().info(f"Sending {direction} effort: {effort}")

        msg = Float64()
        msg.data = effort
        
        # Apply effort repeatedly for smooth actuation
        for _ in range(int(self.duration_sec * 10)):
            self.slider_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

        response.success = True
        response.message = f"Mechanism {direction} command sent."
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ToggleMechanismService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
