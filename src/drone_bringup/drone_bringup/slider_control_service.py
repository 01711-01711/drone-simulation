import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import Float64

class SliderControlService(Node):
    def __init__(self):
        super().__init__('slider_control_service')
        self.publisher = self.create_publisher(Float64, '/column_to_slider/effort_command', 10)
        self.srv = self.create_service(SetBool, 'toggle_mechanism', self.toggle_callback)

    def toggle_callback(self, request, response):
        force = 30.0 if request.data else -30.0
        msg = Float64()
        msg.data = force
        for _ in range(10):  # send multiple effort commands for better actuation
            self.publisher.publish(msg)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))
        response.success = True
        response.message = f"Mechanism {'closed' if request.data else 'opened'}"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = SliderControlService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
