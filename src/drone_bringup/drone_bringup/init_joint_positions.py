import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class EffortInitPublisher(Node):
    def __init__(self):
        super().__init__('init_effort_publisher')

        # List of (joint topic name, effort value)
        self.effort_commands = [
            ('column_to_slider', 0.0),  # Prismatic 
            ('ab1__slider_to_brace_1_joint', 2.0),
            ('ab2__slider_to_brace_2_joint', 2.0),
            ('ab3__slider_to_brace_3_joint', 2.0),
            ('ab4__slider_to_brace_4_joint', 2.0),
            ('ab1__brace_1_to_arm_1_dummy_joint', -1.5),
            ('ab2__brace_2_to_arm_2_dummy_joint', -1.5),
            ('ab3__brace_3_to_arm_3_dummy_joint', -1.5),
            ('ab4__brace_4_to_arm_4_dummy_joint', -1.5),
        ]

        self.joint_publishers = []
        for joint_name, effort in self.effort_commands:
            topic = f"/{joint_name}/effort_command"
            pub = self.create_publisher(Float64, topic, 10)
            self.joint_publishers.append((pub, effort))

        self.publish_count = 0
        self.max_publish_count = 10
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        for pub, effort in self.joint_publishers:
            pub.publish(Float64(data=effort))
        self.get_logger().info(f"Published effort commands, round {self.publish_count + 1}")

        self.publish_count += 1
        if self.publish_count >= self.max_publish_count:
            self.get_logger().info("Effort commands sent enough times. Node shutting down.")
            self.timer.cancel()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = EffortInitPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
