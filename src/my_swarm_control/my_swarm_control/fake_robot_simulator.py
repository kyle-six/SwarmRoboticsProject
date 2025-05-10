import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseWithCovariance, Twist, TwistWithCovariance
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import Header
import math
import time

class FakeRobotSimulator(Node):
    def __init__(self):
        super().__init__('fake_robot_simulator')

        self.robot_names = ['robot1', 'robot2', 'robot3']
        self.publishers = {}
        self.wifi_publishers = {}
        self.positions = {
            'robot1': [0.0, 0.0],
            'robot2': [1.0, 1.0],
            'robot3': [2.0, 2.0]
        }

        for name in self.robot_names:
            self.publishers[name] = self.create_publisher(Odometry, f'/{name}/odom', 10)
            self.wifi_publishers[name] = self.create_publisher(DiagnosticArray, f'/{name}/wifi_neighbors', 10)

        self.timer = self.create_timer(0.5, self.publish_fake_data)

    def publish_fake_data(self):
        now = self.get_clock().now().to_msg()
        for name in self.robot_names:
            x, y = self.positions[name]

            # Update position slightly to simulate movement
            self.positions[name][0] += 0.05
            self.positions[name][1] += 0.05

            # Publish Odometry
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = f'{name}/odom'
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation.z = 0.0
            odom.pose.pose.orientation.w = 1.0
            self.publishers[name].publish(odom)

            # Publish WiFi data (simulate -40 to -60 dBm)
            diag = DiagnosticArray()
            diag.header.stamp = now
            status = DiagnosticStatus()
            status.name = f'{name}_wifi'
            status.level = 0
            status.message = 'WiFi neighbors'
            for other in self.robot_names:
                if other != name:
                    kv = KeyValue()
                    kv.key = other
                    kv.value = str(-40 - abs(hash(name + other)) % 20)
                    status.values.append(kv)
            diag.status.append(status)
            self.wifi_publishers[name].publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

