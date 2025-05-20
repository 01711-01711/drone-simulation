import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math
import time

class DummyImuPublisher(Node):
    def __init__(self):
        super().__init__('dummy_imu_publisher')
        self.publisher_ = self.create_publisher(Imu, '/imu_plugin/out', 10)
        timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        imu_msg = Imu()

        # Simulate level orientation (identity quaternion)
        imu_msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Zero angular velocity and linear acceleration
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 9.81  # gravity

        self.publisher_.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
