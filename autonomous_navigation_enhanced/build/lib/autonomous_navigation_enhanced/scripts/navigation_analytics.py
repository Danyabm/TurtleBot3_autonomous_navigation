#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import time

class NavigationAnalytics(Node):
    """
    Collects and analyzes navigation performance metrics
    - Path efficiency (actual vs optimal distance)
    - Average speed
    - Obstacle avoidance count
    - Time to goal
    """
    def __init__(self):
        super().__init__('navigation_analytics')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        # Analytics data
        self.start_time = None
        self.goal_position = None
        self.start_position = None
        self.current_position = None
        self.total_distance = 0.0
        self.previous_position = None
        self.speeds = []
        self.planned_path_length = 0.0
        
        # Timer for periodic reports
        self.create_timer(5.0, self.print_analytics)
        
        self.get_logger().info('Navigation Analytics initialized!')
    
    def odom_callback(self, msg):
        """Track robot position and calculate metrics"""
        current = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        self.current_position = current
        
        if self.previous_position is not None:
            distance = np.linalg.norm(current - self.previous_position)
            self.total_distance += distance
            
            # Calculate speed
            velocity = msg.twist.twist.linear.x
            self.speeds.append(velocity)
        
        self.previous_position = current
    
    def goal_callback(self, msg):
        """New goal received - reset metrics"""
        self.goal_position = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        
        if self.current_position is not None:
            self.start_position = self.current_position.copy()
            self.start_time = time.time()
            self.total_distance = 0.0
            self.speeds = []
            self.get_logger().info('New goal received - starting analytics')
    
    def path_callback(self, msg):
        """Calculate planned path length"""
        if len(msg.poses) < 2:
            return
        
        length = 0.0
        for i in range(len(msg.poses) - 1):
            p1 = np.array([
                msg.poses[i].pose.position.x,
                msg.poses[i].pose.position.y
            ])
            p2 = np.array([
                msg.poses[i+1].pose.position.x,
                msg.poses[i+1].pose.position.y
            ])
            length += np.linalg.norm(p2 - p1)
        
        self.planned_path_length = length
    
    def print_analytics(self):
        """Print current navigation analytics"""
        # Print basic info even without a goal
        if self.current_position is None:
            self.get_logger().info('Waiting for odometry data...')
            return
            
        if self.goal_position is None or self.start_position is None:
            self.get_logger().info(f'Current Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f})')
            self.get_logger().info('Waiting for goal to start tracking...')
            return
   
            
        # Calculate metrics
        straight_line_distance = np.linalg.norm(self.goal_position - self.start_position)
        remaining_distance = np.linalg.norm(self.goal_position - self.current_position)
        
        efficiency = (straight_line_distance / self.total_distance * 100) if self.total_distance > 0 else 0
        avg_speed = np.mean(self.speeds) if len(self.speeds) > 0 else 0
        
        elapsed_time = time.time() - self.start_time if self.start_time else 0
        
        self.get_logger().info('='*50)
        self.get_logger().info('NAVIGATION ANALYTICS')
        self.get_logger().info(f'Elapsed Time: {elapsed_time:.1f}s')
        self.get_logger().info(f'Distance Traveled: {self.total_distance:.2f}m')
        self.get_logger().info(f'Straight-Line Distance: {straight_line_distance:.2f}m')
        self.get_logger().info(f'Path Efficiency: {efficiency:.1f}%')
        self.get_logger().info(f'Average Speed: {avg_speed:.2f}m/s')
        self.get_logger().info(f'Distance to Goal: {remaining_distance:.2f}m')
        self.get_logger().info(f'Planned Path Length: {self.planned_path_length:.2f}m')
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationAnalytics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()