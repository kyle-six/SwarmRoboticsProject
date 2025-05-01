import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import threading
import platform

# If not an RPI, adapter module redirects to mock GPIO object
from my_robot_hardware.gpio_adapter import GPIO


class MotorController(Node):
    
    _GEAR_REDUCTION_ULN28BYJ_48 = 64     # 64:1 
    _FULL_STEPS_PER_REV_ULN28BYJ_48 = 32
    
    CMD_LOCK = threading.Lock()
    
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
        GPIO.setmode(GPIO.BCM)

        # Define pins (replace with your real pin numbers later)
        self.left_motor_pins = [19, 20, 21, 26]   # GPIO pins for left unipolar motor: (A,B,C,D)
        self.right_motor_pins = [17, 18, 22, 23]  # GPIO pins for right unipolar motor

        for pin in self.left_motor_pins + self.right_motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)

        # 28BYJ-48 full steps for more speed
        # Stepper motor sequence 
        self.step_sequence = [
            [1,0,0,0],
            #[1,1,0,0], # uncomment half-step sequence for smoother movement
            [0,1,0,0],
            #[0,1,1,0],
            [0,0,1,0],
            #[0,0,1,1],
            [0,0,0,1],
            #[1,0,0,1]
        ]
        self.steps_len = len(self.step_sequence)
        self.steps_per_revolution = self._FULL_STEPS_PER_REV_ULN28BYJ_48 * self._GEAR_REDUCTION_ULN28BYJ_48 #64 steps, but 1/64 gear reduction

        # Physical parameters
        self.wheel_radius = 0.024  # meters
        self.wheel_separation = 0.070  # meters

        # Motor speed targets
        self.left_speed = 0.0  # steps per second
        self.right_speed = 0.0  # steps per second

        # Thread for motor control
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z * 100

        # Differential drive kinematics
        v_left = linear_x - (angular_z * self.wheel_separation / 2.0)
        v_right = linear_x + (angular_z * self.wheel_separation / 2.0)

        with self.CMD_LOCK:
             # Convert m/s to steps/s
            self.left_speed = self.mps_to_steps_per_sec(v_left)
            self.right_speed = self.mps_to_steps_per_sec(v_right)

        #self.get_logger().info(f"CmdVel received: v_left = {v_left:.2f} m/s, v_right = {v_right:.2f} m/s")

    def mps_to_steps_per_sec(self, velocity_mps):
        wheel_circumference = 2 * 3.1416 * self.wheel_radius  # meters
        revolutions_per_sec = velocity_mps / wheel_circumference
        return revolutions_per_sec * self.steps_per_revolution

    def step_motor(self, motor_pins, step_index):
        sequence = self.step_sequence[step_index % self.steps_len]
        for pin, val in zip(motor_pins, sequence):
            GPIO.output(pin, val)


    def control_loop(self):
        left_step = 0
        right_step = 0
        last_left_time = time.time()
        last_right_time = time.time()

        while self.running:
            now = time.time()
            # Left motor stepping
            with self.CMD_LOCK:
                if abs(self.left_speed) > 1.0:
                    left_interval = 1.0 / abs(self.left_speed)
                    if (now - last_left_time) >= left_interval:
                        self.step_motor(self.left_motor_pins, left_step)
                        left_step += 1 if self.left_speed > 0 else -1
                        last_left_time = now
            now = time.time()
            # Right motor stepping
            with self.CMD_LOCK:
                if abs(self.right_speed) > 1.0:
                    right_interval = 1.0 / abs(self.right_speed)
                    if (now - last_right_time) >= right_interval:
                        self.step_motor(self.right_motor_pins, right_step)
                        right_step += 1 if self.right_speed > 0 else -1
                        last_right_time = now

            # Small sleep to reduce CPU usage
            #time.sleep(0.001)
            #self.get_logger().info(f"Left speed: {self.left_speed:.1f}, Right speed: {self.right_speed:.1f}")

    def destroy_node(self):
        self.running = False
        self.control_thread.join()
        GPIO.cleanup()
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
