import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu

class DronePIDController(Node):
    def __init__(self):
        super().__init__('drone_pid_controller')

        # Publish to the topics that your thrust plugin listens to
        self.thrust_publishers = [
            self.create_publisher(Float64, f'/rotor{i}/thrust', 10)
            for i in range(1, 5)
        ]

        self.subscription = self.create_subscription(
            Imu,
            '/imu_plugin/out',
            self.imu_callback,
            10
        )

        # PID parameters
        self.kp = 1.5
        self.ki = 0.0
        self.kd = 0.1

        self.target_z = 9.81
        self.prev_error = 0.0
        self.integral = 0.0

    def imu_callback(self, msg: Imu):
        z_accel = msg.linear_acceleration.z

        error = self.target_z - z_accel
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error

        desired_thrust_total = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Equal thrust to 4 rotors
        per_rotor_thrust = max(desired_thrust_total / 4.0, 0.0)

        self.get_logger().info(f"Publishing thrust: {per_rotor_thrust:.4f} to each rotor")

        for pub in self.thrust_publishers:
            pub.publish(Float64(data=per_rotor_thrust))

def main(args=None):
    rclpy.init(args=args)
    node = DronePIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
