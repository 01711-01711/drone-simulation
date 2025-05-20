import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel
import sys

class MechanismGUI(Node):
    def __init__(self):
        super().__init__('mechanism_gui_node')
        self.cli = self.create_client(SetBool, 'toggle_mechanism')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for toggle_mechanism service...')

    def call_service(self, close: bool):
        req = SetBool.Request()
        req.data = close
        future = self.cli.call_async(req)

        # Timeout safeguard
        if not rclpy.spin_until_future_complete(self, future, timeout_sec=3.0):
            self.get_logger().error("Service call timed out.")
            return None

        if future.result() is None:
            self.get_logger().error("Service call returned None.")
        else:
            self.get_logger().info(f"Service responded: {future.result().message}")
    
        return future.result()

class App(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.initUI()

    def initUI(self):
        self.setWindowTitle('Mechanism Control')
        layout = QVBoxLayout()

        self.label = QLabel('Current action: none')
        layout.addWidget(self.label)

        self.btn_close = QPushButton('Close Mechanism')
        self.btn_close.clicked.connect(self.close_mechanism)
        layout.addWidget(self.btn_close)

        self.btn_open = QPushButton('Open Mechanism')
        self.btn_open.clicked.connect(self.open_mechanism)
        layout.addWidget(self.btn_open)

        self.setLayout(layout)
        self.resize(300, 100)
        self.show()

    def close_mechanism(self):
        result = self.ros_node.call_service(True)
        if result and result.success:
            self.label.setText('Current action: closed')
            print("Mechanism closed.")
        else:
            self.label.setText('Failed to close.')
            print("Failed to close.")

    def open_mechanism(self):
        result = self.ros_node.call_service(False)
        if result and result.success:
            self.label.setText('Current action: opened')
            print("Mechanism opened.")
        else:
            self.label.setText('Failed to open.')
            print("Failed to open.")


def main(args=None):
    rclpy.init(args=args)
    ros_node = MechanismGUI()

    app = QApplication(sys.argv)
    gui = App(ros_node)
    app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
