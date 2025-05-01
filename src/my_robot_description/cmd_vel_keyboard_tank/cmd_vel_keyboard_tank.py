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
        self.get_logger().info("Use WASD to move, SPACE to stop, +/- to change speed, Q to quit.")

        # Terminal setup
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        # Initial speeds (IN METERS/SEC)
        self.linear_speed = 0.01 #10mm/sec
        self.angular_speed = 0.05 #10mm/sec
        self.speed_step = 0.005

        self.current_twist = Twist()

    def get_key(self, timeout=0.02):
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1)
        return None

    def stop_robot(self):
        self.publisher.publish(Twist())
        self.get_logger().info("Robot stopped.")

    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()

                new_twist = self.current_twist
                velocity_changed = False

                if key:
                    key = key.lower()
                    velocity_changed = True

                    if key == 'w':
                        new_twist.linear.x = self.linear_speed
                    elif key == 's':
                        new_twist.linear.x = -self.linear_speed
                    elif key == 'a':
                        new_twist.angular.z = -self.angular_speed
                    elif key == 'd':
                        new_twist.angular.z = self.angular_speed
                    elif key == '+' or key == '=':
                        self.linear_speed += self.speed_step
                        self.angular_speed += self.speed_step
                        #self.get_logger().info(f"Increased speed: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}")
                        velocity_changed = False  # no movement, just speed change
                    elif key == '-' or key == '_':
                        self.linear_speed = max(0.0, self.linear_speed - self.speed_step)
                        self.angular_speed = max(0.0, self.angular_speed - self.speed_step)
                        #self.get_logger().info(f"Decreased speed: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}")
                        velocity_changed = False
                    elif key == " ":
                        self.current_twist = Twist()
                    elif key == 'q':
                        self.get_logger().info("Quitting.")
                        break
                    else:
                        velocity_changed = False  # Unknown key

                    if velocity_changed:
                        self.current_twist = new_twist

                self.publisher.publish(self.current_twist)
                self.get_logger().info(f"linear={self.current_twist.linear.x:.3f}, angular={self.current_twist.angular.z:.3f}")

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
