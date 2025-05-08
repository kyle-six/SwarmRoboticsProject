import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import QoSProfile

class WiFiNeighborsAggregator(Node):
    def __init__(self):
        super().__init__('wifi_neighbors_aggregator')
        self._subscriptions = {}
        self.neighbor_data = {}

        self.robot_names = ['robot1', 'robot2', 'robot3']  # Add more as needed
        qos = QoSProfile(depth=10)

        for robot in self.robot_names:
            topic = f'/{robot}/wifi_neighbors'
            self._subscriptions[robot] = self.create_subscription(
                DiagnosticArray,
                topic,
                lambda msg, robot=robot: self.handle_neighbors(msg, robot),
                qos
            )
            self.get_logger().info(f'Subscribed to {topic}')

        self.timer = self.create_timer(5.0, self.log_neighbors)

    def handle_neighbors(self, msg, robot):
        self.neighbor_data[robot] = msg

    def log_neighbors(self):
        self.get_logger().info("=== Aggregated WiFi Neighbors ===")
        for robot, msg in self.neighbor_data.items():
            for status in msg.status:
                neighbors = [f"{kv.key}:{kv.value}" for kv in status.values]
                self.get_logger().info(f"{robot} sees: {', '.join(neighbors)}")

def main(args=None):
    rclpy.init(args=args)
    node = WiFiNeighborsAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# if __name__ == '__main__':
#     main()