#!/usr/bin/env python3
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class TurtleBotRLEnv(gym.Env, Node):
    """
    Gymnasium Environment for TurtleBot3 RL Navigation
    
    Observation Space: 24 laser scan readings + 2 goal relative position
    Action Space: Linear velocity (0-0.22 m/s) + Angular velocity (-2 to 2 rad/s)
    
    Reward Function:
    - Positive: Moving toward goal, reaching goal
    - Negative: Collision, moving away from goal
    """
    
    metadata = {'render_modes': []}
    
    def __init__(self, goal_position=(2.0, 2.0)):
        # Initialize ROS2 Node
        rclpy.init()
        Node.__init__(self, 'rl_navigation_env')
        
        # Goal
        self.goal_position = np.array(goal_position)
        self.goal_threshold = 0.3  # meters
        
        # State variables
        self.laser_data = None
        self.robot_position = np.array([0.0, 0.0])
        self.robot_yaw = 0.0
        self.prev_distance_to_goal = None
        self.collision = False
        self.goal_reached = False
        
        # Episode tracking
        self.episode_steps = 0
        self.max_episode_steps = 500
        
        # ROS2 Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Define action and observation space
        # Actions: [linear_velocity, angular_velocity]
        self.action_space = spaces.Box(
            low=np.array([0.0, -2.0]),
            high=np.array([0.22, 2.0]),
            dtype=np.float32
        )
        
        # Observations: 24 laser readings + [dx, dy] to goal
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(26,),
            dtype=np.float32
        )
        
        self.get_logger().info('🤖 RL Environment initialized!')
    
    def laser_callback(self, msg):
        """Process laser scan data"""
        # Downsample 360 readings to 24 (every 15 degrees)
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 3.5  # Max range
        ranges[np.isnan(ranges)] = 0.0
        
        # Sample 24 readings evenly
        indices = np.linspace(0, len(ranges)-1, 24, dtype=int)
        self.laser_data = ranges[indices]
        
        # Check collision (any reading < 0.2m)
        if np.min(self.laser_data) < 0.2:
            self.collision = True
    
    def odom_callback(self, msg):
        """Update robot position"""
        self.robot_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        # Extract yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def get_observation(self):
        """
        Construct observation vector:
        [24 laser readings, dx_to_goal, dy_to_goal]
        """
        # Wait for sensor data
        while self.laser_data is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Relative goal position in robot frame
        dx = self.goal_position[0] - self.robot_position[0]
        dy = self.goal_position[1] - self.robot_position[1]
        
        # Rotate to robot's local frame
        dx_local = dx * math.cos(-self.robot_yaw) - dy * math.sin(-self.robot_yaw)
        dy_local = dx * math.sin(-self.robot_yaw) + dy * math.cos(-self.robot_yaw)
        
        observation = np.concatenate([
            self.laser_data,
            [dx_local, dy_local]
        ]).astype(np.float32)
        
        return observation
    
    def calculate_reward(self):
        """
        Reward function - the heart of RL learning!
        """
        reward = 0.0
        
        # Distance to goal
        current_distance = np.linalg.norm(self.goal_position - self.robot_position)
        
        # 1. Goal reached - BIG positive reward
        if current_distance < self.goal_threshold:
            self.goal_reached = True
            reward += 200.0
            self.get_logger().info('🎉 GOAL REACHED!')
            return reward, True  # Episode done
        
        # 2. Collision - BIG negative reward
        if self.collision:
            reward -= 100.0
            self.get_logger().info('💥 COLLISION!')
            return reward, True  # Episode done
        
        # 3. Progress toward goal - positive reward
        if self.prev_distance_to_goal is not None:
            progress = self.prev_distance_to_goal - current_distance
            reward += progress * 10.0  # Scale up the reward
        
        # 4. Distance penalty - encourage efficiency
        reward -= current_distance * 0.1
        
        # 5. Time penalty - don't waste time
        reward -= 0.1
        
        # 6. Forward motion bonus (discourage spinning in place)
        reward += 0.5
        
        self.prev_distance_to_goal = current_distance
        
        # 7. Timeout
        if self.episode_steps >= self.max_episode_steps:
            self.get_logger().info('⏰ Episode timeout')
            return reward, True
        
        return reward, False
    
    def step(self, action):
        """
        Execute action and return (observation, reward, done, truncated, info)
        """
        # Publish action
        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        self.cmd_vel_pub.publish(cmd)
        
        # Wait for simulation to update
        time.sleep(0.1)
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Get new observation
        observation = self.get_observation()
        
        # Calculate reward
        reward, done = self.calculate_reward()
        
        # Increment step counter
        self.episode_steps += 1
        
        info = {
            'distance_to_goal': np.linalg.norm(self.goal_position - self.robot_position),
            'collision': self.collision,
            'goal_reached': self.goal_reached
        }
        
        return observation, reward, done, False, info
    
    def reset(self, seed=None, options=None):
        """Reset environment to initial state"""
        super().reset(seed=seed)
        
        # Stop robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        
        # Reset state
        self.collision = False
        self.goal_reached = False
        self.episode_steps = 0
        self.prev_distance_to_goal = None
        
        # Wait for stabilization
        time.sleep(0.5)
        
        # Get initial observation
        observation = self.get_observation()
        
        self.get_logger().info('🔄 Environment reset')
        
        return observation, {}
    
    def close(self):
        """Cleanup"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.destroy_node()
        rclpy.shutdown()

def main():
    """Test environment"""
    env = TurtleBotRLEnv(goal_position=(2.0, 2.0))
    
    obs, info = env.reset()
    print(f"Observation shape: {obs.shape}")
    print(f"Action space: {env.action_space}")
    
    # Test random actions
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, done, truncated, info = env.step(action)
        print(f"Step {i}: Reward={reward:.2f}, Done={done}")
        
        if done:
            obs, info = env.reset()
    
    env.close()

if __name__ == '__main__':
    main()