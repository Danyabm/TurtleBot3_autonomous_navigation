#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
from collections import deque

class SmartGoalPlanner(Node):
    """
    Intelligent goal planning that analyzes map obstacle density
    and suggests optimal exploration targets
    """
    def __init__(self):
        super().__init__('smart_goal_planner')
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # Publisher for goal poses
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.map_data = None
        self.map_info = None
        
        # Create timer for autonomous goal generation
        self.timer = self.create_timer(10.0, self.generate_smart_goal)
        
        self.get_logger().info('Smart Goal Planner initialized!')
    
    def map_callback(self, msg):
        """Store map data for analysis"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info
    
    def calculate_obstacle_density(self, x, y, radius=10):
        """Calculate obstacle density around a point"""
        if self.map_data is None:
            return 0.0
        
        x_indices = np.arange(max(0, x-radius), min(self.map_data.shape[1], x+radius))
        y_indices = np.arange(max(0, y-radius), min(self.map_data.shape[0], y+radius))
        
        region = self.map_data[np.ix_(y_indices, x_indices)]
        occupied = np.sum(region > 50)  # Cells with >50% occupancy
        total = region.size
        
        return occupied / total if total > 0 else 0.0
    
    def find_exploration_frontiers(self):
        """Find unexplored areas (frontiers) on the map"""
        if self.map_data is None:
            return []
        
        # Find cells that are unknown (-1) and adjacent to free space (0)
        frontiers = []
        
        for i in range(1, self.map_data.shape[0]-1):
            for j in range(1, self.map_data.shape[1]-1):
                if self.map_data[i, j] == -1:  # Unknown cell
                    # Check if adjacent to free space
                    neighbors = self.map_data[i-1:i+2, j-1:j+2]
                    if np.any(neighbors == 0):
                        # Calculate score based on obstacle density
                        density = self.calculate_obstacle_density(j, i)
                        score = 1.0 - density  # Prefer low-density areas
                        frontiers.append((j, i, score))
        
        return sorted(frontiers, key=lambda x: x[2], reverse=True)[:5]
    
    def generate_smart_goal(self):
            """Generate an intelligent goal based on map analysis"""
            if self.map_data is None or self.map_info is None:
                self.get_logger().info('Waiting for map data...')
                return
            
            # Try to find frontiers first
            frontiers = self.find_exploration_frontiers()
            
            if frontiers:
                # Use frontier-based exploration
                best_frontier = frontiers[0]
                map_x, map_y, score = best_frontier
                self.get_logger().info(f'Using frontier exploration')
            else:
                # Fall back to free space exploration
                self.get_logger().info('No frontiers - using free space exploration')
                
                # Find free space cells
                free_cells = np.argwhere((self.map_data >= 0) & (self.map_data < 50))
                
                if len(free_cells) == 0:
                    self.get_logger().info('No free space found on map!')
                    return
                
                # Sample and score candidates
                candidates = []
                sample_size = min(100, len(free_cells))
                indices = np.random.choice(len(free_cells), sample_size, replace=False)
                
                for idx in indices:
                    map_y, map_x = free_cells[idx]
                    density = self.calculate_obstacle_density(map_x, map_y)
                    score = 1.0 - density
                    candidates.append((map_x, map_y, score))
                
                # Select best candidate
                best_candidate = max(candidates, key=lambda x: x[2])
                map_x, map_y, score = best_candidate
            
            # Convert map coordinates to world coordinates
            world_x = map_x * self.map_info.resolution + self.map_info.origin.position.x
            world_y = map_y * self.map_info.resolution + self.map_info.origin.position.y
            
            # Create and publish goal
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = world_x
            goal.pose.position.y = world_y
            goal.pose.orientation.w = 1.0
        
            self.goal_pub.publish(goal)
            self.get_logger().info(f'Published smart goal: ({world_x:.2f}, {world_y:.2f}) with score {score:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = SmartGoalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()