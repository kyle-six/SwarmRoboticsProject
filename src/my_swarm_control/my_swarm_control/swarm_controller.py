import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SwarmController(Node):
    def __init__(self, robot_names):
        super().__init__('swarm_controller')

        self.robot_names = robot_names
        self.cmd_publishers = {}
        self.odom_subscribers = {}

        for name in robot_names:
            # Create publisher for cmd_vel
            pub = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.cmd_publishers[name] = pub

            # Create subscriber for odom
            sub = self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, robot=name: self.odom_callback(robot, msg),
                10
            )
            self.odom_subscribers[name] = sub

        self.timer = self.create_timer(0.5, self.control_loop)

        self.latest_poses = {name: None for name in robot_names}

    def odom_callback(self, robot, msg):
        self.latest_poses[robot] = msg.pose.pose

    def control_loop(self):
        for name in self.robot_names:
            twist = Twist()
            # Basic example: Move forward if we have pose data
            if self.latest_poses[name] is not None:
                twist.linear.x = 0.1
            else:
                twist.linear.x = 0.0

            self.cmd_publishers[name].publish(twist)


def main(args=None):
    rclpy.init(args=args)
    robot_names = ['robot1', 'robot2']
    controller = SwarmController(robot_names)
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
