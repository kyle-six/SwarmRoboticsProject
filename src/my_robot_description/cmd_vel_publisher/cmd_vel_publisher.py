import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

class CmdVelKeyboard(Node):
    def __init__(self):
        super().__init__('cmd_vel_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("Use W/A/S/D keys to move. Press Q to quit.")

        # Setup terminal
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Settings
        self.linear_speed = 0.2
        self.angular_speed = 0.5

        self.active_twist = Twist()
        self.last_key_time = self.get_clock().now()
        
        self.io_timeout = 0.02
        self.debounce_period = 0.5

    def get_key(self, timeout=0.02):
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None

    def stop_robot(self):
        stop_twist = Twist()
        self.publisher.publish(stop_twist)
        self.get_logger().info("Robot stopped.")

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key(timeout=self.io_timeout)

                new_twist = Twist()

                if key:
                    if key.lower() == 'w':
                        new_twist.linear.x = self.linear_speed
                    elif key.lower() == 's':
                        new_twist.linear.x = -self.linear_speed
                    elif key.lower() == 'a':
                        new_twist.angular.z = self.angular_speed
                    elif key.lower() == 'd':
                        new_twist.angular.z = -self.angular_speed
                    elif key.lower() == 'q':
                        self.get_logger().info("Exiting on user request.")
                        break

                    self.active_twist = new_twist
                    self.last_key_time = self.get_clock().now()

                # Check if key was pressed recently (within 0.2 seconds)
                time_since_last_key = (self.get_clock().now() - self.last_key_time).nanoseconds * 1e-9

                if time_since_last_key > self.debounce_period:
                    # No key pressed for a while, stop the robot
                    self.active_twist = Twist()

                # Always publish
                self.publisher.publish(self.active_twist)
                self.get_logger().info(f"Sent: linear.x = {self.active_twist.linear.x:.2f}, angular.z = {self.active_twist.angular.z:.2f}")

        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received. Exiting.")
        finally:
            self.stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
