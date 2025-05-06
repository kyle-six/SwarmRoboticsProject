import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import subprocess


class WifiMeshNode(Node):
    def __init__(self):
        super().__init__('wifi_mesh_node')

        self.interface = 'wlan0'
        self.mesh_iface = 'mesh0'
        self.bat_iface = 'bat0'
        self.mesh_id = 'my_robot_mesh'

        self.status_pub = self.create_publisher(String, 'wifi_mesh/status', 10)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.timer = self.create_timer(5.0, self.publish_status)

        self.get_logger().info("WiFi Mesh Node (batman-adv) started.")
        self.setup_mesh_interface()

    def run_cmd(self, cmd, check=True):
        try:
            self.get_logger().debug(f"Running: {' '.join(cmd)}")
            subprocess.run(cmd, check=check, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Command failed: {' '.join(cmd)}\n{e.stderr.decode()}")
            raise

    def setup_mesh_interface(self):
        self.get_logger().info("Setting up mesh interface with batman-adv...")

        try:
            # Ensure kernel module is loaded
            self.run_cmd(['sudo', 'modprobe', 'batman-adv'])

            # Create mesh interface and set up mesh mode
            self.run_cmd(['sudo', 'iw', 'dev', self.interface, 'interface', 'add', self.mesh_iface, 'type', 'mp'])
            self.run_cmd(['sudo', 'ip', 'link', 'set', self.mesh_iface, 'up'])
            self.run_cmd(['sudo', 'iw', self.mesh_iface, 'meshid', self.mesh_id])
            self.run_cmd(['sudo', 'iw', self.mesh_iface, 'set', 'mesh_param', 'mesh_fwding=1'])

            # Attach mesh0 to bat0
            self.run_cmd(['sudo', 'batctl', 'if', 'add', self.mesh_iface])
            self.run_cmd(['sudo', 'ip', 'link', 'set', self.bat_iface, 'up'])

            self.get_logger().info("Mesh interface configured successfully.")

        except Exception as e:
            self.get_logger().error(f"Mesh setup failed: {e}")

    def get_batman_status(self):
        try:
            result = subprocess.check_output(['batctl', 'n'], text=True)
            return result.strip()
        except subprocess.CalledProcessError as e:
            return f"Error checking batman-adv neighbors: {e}"

    def publish_status(self):
        status_str = self.get_batman_status()

        # Publish plain string status
        status_msg = String()
        status_msg.data = status_str
        self.status_pub.publish(status_msg)

        # Publish diagnostic status
        diag = DiagnosticArray()
        stat = DiagnosticStatus()
        stat.name = "batman-adv mesh status"
        stat.level = DiagnosticStatus.OK
        stat.message = "batctl n output"
        stat.values = [KeyValue(key='Neighbors', value=status_str)]
        diag.status.append(stat)

        self.diagnostic_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = WifiMeshNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
