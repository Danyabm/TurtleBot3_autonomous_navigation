#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
from collections import deque

class DynamicObstaclePredictor(Node):
    """
    Predicts moving obstacles and publishes warnings
    Uses velocity estimation from consecutive laser scans
    """
    def __init__(self):
        super().__init__('obstacle_predictor')
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.warning_pub = self.create_publisher(
            String,
            '/obstacle_warnings',
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/emergency_stop',
            10
        )
        
        # Store recent scans for motion detection
        self.scan_history = deque(maxlen=5)
        self.danger_threshold = 0.5  # meters
        self.emergency_threshold = 0.3  # meters
        
        self.get_logger().info('Dynamic Obstacle Predictor initialized!')
    
    def scan_callback(self, msg):
        """Process laser scan and detect dynamic obstacles"""
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        
        self.scan_history.append(ranges)
        
        if len(self.scan_history) < 2:
            return
        
        # Calculate change in distances (simple motion detection)
        current = self.scan_history[-1]
        previous = self.scan_history[-2]
        
        delta = previous - current  # Positive = approaching
        approaching_indices = np.where(delta > 0.05)[0]  # Moving toward us
        
        if len(approaching_indices) > 0:
            closest_distance = np.min(current[approaching_indices])
            velocity = np.mean(delta[approaching_indices]) * 10  # Approximate velocity
            
            if closest_distance < self.emergency_threshold:
                self.emergency_stop()
                warning = f'EMERGENCY: Dynamic obstacle at {closest_distance:.2f}m!'
            elif closest_distance < self.danger_threshold:
                warning = f'WARNING: Moving obstacle detected at {closest_distance:.2f}m, velocity {velocity:.2f}m/s'
            else:
                warning = f'INFO: Tracking moving obstacle at {closest_distance:.2f}m'
            
            msg = String()
            msg.data = warning
            self.warning_pub.publish(msg)
            self.get_logger().info(warning)
    
    def emergency_stop(self):
        """Publish emergency stop command"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstaclePredictor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()