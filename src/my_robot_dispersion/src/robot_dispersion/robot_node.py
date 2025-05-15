#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

class DispersionRobot(Node):
    def __init__(self, robot_id):
        super().__init__(f'robot_{robot_id}_node')
        self.robot_id = robot_id
        
        # Parameters
        self.lower_threshold = 5.0  # meters
        self.upper_threshold = 8.0  # meters
        self.robot_speed = 0.2      # m/s
        self.turn_rate = 0.5        # rad/s (≈28.6°/sec)
        self.turn_duration = 1.57   # seconds for 45° turn (0.5 rad/s * 1.57 ≈ 45°)
        
        # State
        self.frozen = False
        self.distances = {}
        self.turning = False
        self.turn_start_time = None
        
        # Publishers/Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, f'robot_{self.robot_id}/cmd_vel', 10)
        self.create_subscription(Float32MultiArray, '/rssi_distances', self.rssi_callback, 10)
        
        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f'Robot {robot_id} initialized')

    def rssi_callback(self, msg):
        self.distances = {i: msg.data[i] for i in range(3) if i != self.robot_id}

    def control_loop(self):
        if self.frozen or not self.distances:
            self.stop_movement()
            return
            
        other_dists = list(self.distances.values())
        
        # Freeze if reached threshold with any robot
        if any(d >= self.lower_threshold for d in other_dists):
            self.frozen = True
            self.get_logger().info(f'Robot {self.robot_id} froze at threshold')
            return
            
        # Check if we're the last active robot
        active_robots = sum(1 for d in other_dists if d < self.lower_threshold)
        is_last_active = active_robots <= 1
        
        cmd = Twist()
        
        if is_last_active:
            # Third robot logic - navigate between thresholds
            if all(self.lower_threshold <= d <= self.upper_threshold for d in other_dists):
                self.frozen = True
                return
                
            # Determine movement direction
            needs = []
            for dist in other_dists:
                if dist < self.lower_threshold:
                    needs.append(-1)  # Move away
                elif dist > self.upper_threshold:
                    needs.append(1)   # Move closer
                else:
                    needs.append(0)   # In range
                    
            if sum(needs) < 0:  # Net need to move away
                cmd.linear.x = self.robot_speed
            elif sum(needs) > 0:  # Net need to move closer
                cmd.linear.x = self.robot_speed
        else:
            # First two robots - move away from each other
            current_rssi = sum(1/(d + 0.001) for d in other_dists)
            
            # Try moving forward
            cmd.linear.x = self.robot_speed
            self.cmd_vel_pub.publish(cmd)
            
            # Check if we're moving away
            rclpy.spin_once(self, timeout_sec=0.3)
            new_rssi = sum(1/(d + 0.001) for d in other_dists)
            
            if new_rssi >= current_rssi:  # Not moving away
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_rate  # Turn left
                self.get_logger().info('Turning to find better direction')
        
        self.cmd_vel_pub.publish(cmd)

    def stop_movement(self):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    robot_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    node = DispersionRobot(robot_id)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import sys
    main(sys.argv)