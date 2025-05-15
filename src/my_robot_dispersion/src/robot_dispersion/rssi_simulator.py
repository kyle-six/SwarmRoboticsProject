#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import numpy as np

class RSSISimulator(Node):
    def __init__(self):
        super().__init__('rssi_simulator')
        self.positions = {
            0: [0.0, 1.0],
            1: [-1.0, -1.0],
            2: [1.0, -1.0]
        }
        self.publisher = self.create_publisher(Float32MultiArray, '/rssi_distances', 10)
        self.timer = self.create_timer(0.1, self.update_distances)
        
    def update_distances(self):
        distances = [0.0, 0.0, 0.0]
        for i in range(3):
            for j in range(i+1, 3):
                dist = math.dist(self.positions[i], self.positions[j])
                distances[i] = dist
                distances[j] = dist
                
        msg = Float32MultiArray()
        msg.data = distances
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RSSISimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()