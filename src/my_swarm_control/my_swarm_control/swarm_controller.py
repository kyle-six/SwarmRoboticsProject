import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import json
import os
from datetime import datetime

# Replace with your actual message type
from my_swarm_interfaces.msg import WifiNeighbors  

class SwarmController(Node):
    def __init__(self, robot_names):
        super().__init__('swarm_controller')
        self.robot_names = robot_names

        self.cmd_publishers = {}
        self.odom_subscribers = {}
        self.wifi_subscribers = {}

        self.latest_odom = {name: None for name in robot_names}
        self.latest_wifi = {name: None for name in robot_names}
        self.log_data = {name: [] for name in robot_names}

        for name in robot_names:
            # cmd_vel publisher
            self.cmd_publishers[name] = self.create_publisher(
                Twist, f'/{name}/cmd_vel', 10
            )

            # odometry subscriber
            self.odom_subscribers[name] = self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, robot=name: self.odom_callback(robot, msg),
                10
            )

            # wifi_neighbors subscriber
            self.wifi_subscribers[name] = self.create_subscription(
                WifiNeighbors,
                f'/{name}/wifi_neighbors',
                lambda msg, robot=name: self.wifi_callback(robot, msg),
                10
            )

        # Control loop at 2 Hz
        self.timer = self.create_timer(0.5, self.control_loop)

        self.navigation_started = False
        self.data_log_dir = f"/tmp/swarm_logs_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        os.makedirs(self.data_log_dir, exist_ok=True)

        self.get_logger().info("Waiting for all robots to be ready...")

    def odom_callback(self, robot, msg):
        self.latest_odom[robot] = msg.pose.pose

    def wifi_callback(self, robot, msg):
        self.latest_wifi[robot] = msg

    def all_robots_ready(self):
        return all(self.latest_odom[name] and self.latest_wifi[name] for name in self.robot_names)

    def prompt_start(self):
        self.get_logger().info("All robots are ready.")
        print("\n>>> Press Enter to start navigation...")
        input()
        self.navigation_started = True
        self.get_logger().info("Navigation started.")

    def control_loop(self):
        if not self.navigation_started:
            if self.all_robots_ready():
                self.prompt_start()
            return

        for name in self.robot_names:
            twist = Twist()
            twist.linear.x = 0.1  # Move forward
            self.cmd_publishers[name].publish(twist)

            # Log data
            self.log_data[name].append({
                "timestamp": self.get_clock().now().to_msg().sec,
                "pose": {
                    "x": self.latest_odom[name].position.x,
                    "y": self.latest_odom[name].position.y
                },
                "wifi": [
                    {"neighbor": n.neighbor, "rssi": n.rssi}
                    for n in self.latest_wifi[name].neighbors
                ]
            })

    def stop_robots(self):
        for name in self.robot_names:
            twist = Twist()
            self.cmd_publishers[name].publish(twist)

    def save_logs(self):
        for name in self.robot_names:
            file_path = os.path.join(self.data_log_dir, f"{name}_log.json")
            with open(file_path, "w") as f:
                json.dump(self.log_data[name], f, indent=2)
            self.get_logger().info(f"Saved log for {name} to {file_path}")

    def shutdown(self):
        self.get_logger().info("Shutting down. Stopping all robots and saving logs...")
        self.stop_robots()
        self.save_logs()


def main(args=None):
    rclpy.init(args=args)
    robot_names = ['robot1', 'robot2', 'robot3']
    controller = SwarmController(robot_names)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.shutdown()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
