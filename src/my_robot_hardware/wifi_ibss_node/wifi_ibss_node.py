import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from datetime import datetime
import csv
import threading


class SwarmController(Node):
    def __init__(self, robot_names):
        super().__init__('swarm_controller')

        self.robot_names = robot_names
        self.cmd_publishers = {}
        self.odom_subscribers = {}
        self.wifi_subscribers = {}
        self.latest_poses = {name: None for name in robot_names}
        self.latest_wifi = {name: [] for name in robot_names}
        self.ready_flags = {name: False for name in robot_names}

        for name in robot_names:
            self.cmd_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)

            self.odom_subscribers[name] = self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, robot=name: self.odom_callback(robot, msg),
                10
            )

            self.wifi_subscribers[name] = self.create_subscription(
                DiagnosticArray,
                f'/{name}/wifi_neighbors',
                lambda msg, robot=name: self.wifi_callback(robot, msg),
                10
            )

        self.timer = self.create_timer(0.5, self.control_loop)

        self.logging_enabled = False
        self.log_file = open('swarm_log.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow([
            'timestamp', 'robot',
            'position_x', 'position_y',
            'orientation_z', 'orientation_w',
            'wifi_info'
        ])

        self.log_duration_sec = None  # Set to a number (e.g., 60) to auto-stop logging
        self.start_time = None

        threading.Thread(target=self.wait_for_start_input, daemon=True).start()

    def odom_callback(self, robot, msg):
        self.latest_poses[robot] = msg.pose.pose
        self.ready_flags[robot] = True

    def wifi_callback(self, robot, msg: DiagnosticArray):
        info = []
        for status in msg.status:
            if status.name.startswith(robot):
                info.extend([f"{kv.key}:{kv.value}" for kv in status.values])
        self.latest_wifi[robot] = info

    def wait_for_start_input(self):
        while not all(self.ready_flags.values()):
            print("[STATUS] Waiting for all robots to be ready...")
            rclpy.spin_once(self, timeout_sec=1.0)
        print("[STATUS] All robots ready.")
        input("Press Enter to START navigation... ")
        self.logging_enabled = True
        self.start_time = datetime.now()
        print("[LOGGING] Logging started.")

    def control_loop(self):
        for name in self.robot_names:
            twist = Twist()

            if self.latest_poses[name] is not None and self.logging_enabled:
                twist.linear.x = 0.1
            else:
                twist.linear.x = 0.0

            self.cmd_publishers[name].publish(twist)

            if self.logging_enabled:
                now = datetime.now()
                timestamp = now.isoformat()
                pos = self.latest_poses[name]
                wifi = ";".join(self.latest_wifi[name])
                if pos:
                    self.csv_writer.writerow([
                        timestamp, name,
                        pos.position.x, pos.position.y,
                        pos.orientation.z, pos.orientation.w,
                        wifi
                    ])
                else:
                    self.csv_writer.writerow([
                        timestamp, name,
                        "N/A", "N/A", "N/A", "N/A",
                        wifi
                    ])
                self.log_file.flush()

                # Stop logging after duration if set
                if self.log_duration_sec is not None:
                    elapsed = (now - self.start_time).total_seconds()
                    if elapsed > self.log_duration_sec:
                        self.logging_enabled = False
                        print("[LOGGING] Logging duration completed.")

    def destroy_node(self):
        for name in self.robot_names:
            stop_twist = Twist()
            self.cmd_publishers[name].publish(stop_twist)
        self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    robot_names = ['robot1', 'robot2', 'robot3']
    controller = SwarmController(robot_names)

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
