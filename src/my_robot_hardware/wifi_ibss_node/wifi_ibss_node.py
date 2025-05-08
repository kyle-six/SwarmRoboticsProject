import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import subprocess
import re
import time

# Example MAC-to-name map
KNOWN_PEERS = {
    'b8:27:eb:ad:bc:32': 'robot1',
    'b8:27:eb:c4:90:4a': 'robot2',
    'b8:27:eb:7c:ed:93': 'robot3',
}

class WifiIBSSNode(Node):
    def __init__(self):
        super().__init__('wifi_ibss_node')
        self.declare_parameter('interface', 'wlan0')
        self.interface = self.get_parameter('interface').get_parameter_value().string_value
        self.publisher = self.create_publisher(DiagnosticArray, 'wifi_neighbors', 10)
        self.timer = self.create_timer(5.0, self.publish_signal_strengths)

        self.get_logger().info(f"WiFi IBSS Node started on interface: {self.interface}")

    def publish_signal_strengths(self):
        neighbors = self.get_signal_strengths()

        diag_msg = DiagnosticArray()
        diag_status = DiagnosticStatus()
        diag_status.name = 'WiFi Signal Strengths'
        diag_status.level = DiagnosticStatus.OK
        diag_status.message = 'Signal levels to IBSS peers'
        diag_status.values = []

        for mac, signal in neighbors.items():
            name = KNOWN_PEERS.get(mac.lower(), mac)
            diag_status.values.append(KeyValue(key=name, value=f"{signal} dBm"))

        diag_msg.status.append(diag_status)
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(diag_msg)

    def get_signal_strengths(self):
        neighbors = {}
        try:
            output = subprocess.check_output(['iwconfig', self.interface], text=True)
            signal_match = re.search(r'Signal level=(-?\d+) dBm', output)
            if signal_match:
                # IBSS mode often only gives one peer’s signal strength
                # if more are connected, use iw dev wlan0 station dump (less reliable in IBSS)
                neighbors['unknown'] = int(signal_match.group(1))
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"iwconfig failed: {e}")
        return neighbors

def main(args=None):
    rclpy.init(args=args)
    node = WifiIBSSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WiFi IBSS node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
