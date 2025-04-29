import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading
import platform

# If not an RPI, adapter module redirects to mock GPIO object
from my_robot_hardware.gpio_adapter import OutputDevice

if platform.system() == 'Linux' and 'raspberrypi' in platform.uname().machine:
    print("Running on a Raspberry Pi system, using real GPIO setup.")
else:
    print("Running on a non-Raspberry Pi system, using mock GPIO setup.")

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Setup ROS 2 subscription
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.left_motor_pins = []
        self.right_motor_pins = []
        self.step_sequence = []

        # GPIO setup
        #GPIO.setmode(GPIO.BCM)

        # Define pins (replace with your real pin numbers later)
        self.left_motor_pins = [17, 18, 22, 23]   # Example GPIO pins for left motor
        self.right_motor_pins = [19, 20, 21, 26]   # Example GPIO pins for right motor
        
        self.left_outputs = []
        for pin in self.left_motor_pins:
            new_output = OutputDevice(pin)
            new_output.off()
            self.left_outputs.append(new_output)
        self.right_outputs = []
        for pin in self.right_motor_pins:
            new_output = OutputDevice(pin)
            new_output.off()
            self.right_outputs.append(new_output)

        # Stepper motor sequence (half-step sequence for smoother movement)
        self.step_sequence = [
            [1,0,0,0],
            [1,1,0,0],
            [0,1,0,0],
            [0,1,1,0],
            [0,0,1,0],
            [0,0,1,1],
            [0,0,0,1],
            [1,0,0,1]
        ]

        # Physical parameters
        self.wheel_radius = 0.048  # meters
        self.wheel_separation = 0.2  # placeholder, meters

        # Motor speed targets
        self.left_speed = 0.0  # steps per second
        self.right_speed = 0.0  # steps per second

        # Thread for motor control
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive kinematics
        v_left = linear_x - (angular_z * self.wheel_separation / 2.0)
        v_right = linear_x + (angular_z * self.wheel_separation / 2.0)

        # Convert m/s to steps/s
        self.left_speed = self.mps_to_steps_per_sec(v_left)
        self.right_speed = self.mps_to_steps_per_sec(v_right)

        self.get_logger().info(f"CmdVel received: v_left = {v_left:.2f} m/s, v_right = {v_right:.2f} m/s")

    def mps_to_steps_per_sec(self, velocity_mps):
        steps_per_revolution = 4096  # 28BYJ-48 full steps
        wheel_circumference = 2 * 3.1416 * self.wheel_radius  # meters
        revolutions_per_sec = velocity_mps / wheel_circumference
        return revolutions_per_sec * steps_per_revolution

    def step_motor(self, motor_outputs, step_index, direction):
        sequence_index = step_index if direction >= 0 else (7 - step_index)
        for output, val in zip(motor_outputs, self.step_sequence[sequence_index]):
            if val == 1:
                output.on()
            else:
                output.off()

    def control_loop(self):
        left_step = 0
        right_step = 0
        update_rate = 0.002  # 2 ms ~ 500 Hz control rate
        while self.running:
            if abs(self.left_speed) > 1.0:
                self.step_motor(self.left_outputs, left_step % 8, self.left_speed)
                left_step += 1 if self.left_speed > 0 else -1
            if abs(self.right_speed) > 1.0:
                self.step_motor(self.right_outputs, right_step % 8, self.right_speed)
                right_step += 1 if self.right_speed > 0 else -1

            # Delay according to speed
            if abs(self.left_speed) > 0.1:
                time.sleep(abs(1.0 / self.left_speed))
            if abs(self.right_speed) > 0.1:
                time.sleep(abs(1.0 / self.right_speed))
                
            time.sleep(update_rate)

    def destroy_node(self):
        self.running = False
        self.control_thread.join()
        for o in self.left_outputs + self.right_outputs:
            o.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
