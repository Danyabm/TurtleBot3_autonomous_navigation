# TurtleBot3_autonomous_navigation

TurtleBot3 Autonomous Navigation with Path Planning

Intelligent autonomous navigation system for TurtleBot3 with smart goal planning, dynamic obstacle prediction, and real-time analytics. Built from scratch while learning ROS2.

**Overview**
This project implements an advanced autonomous navigation system for TurtleBot3 robots in simulated environments. Unlike basic navigation implementations, this system incorporates:

Intelligent Goal Planning: Frontier-based exploration with obstacle density analysis
Predictive Obstacle Avoidance: Real-time detection and prediction of dynamic obstacles
Performance Analytics: Comprehensive metrics tracking for path efficiency and navigation quality

Built as a learning journey from ROS2 basics to advanced robotics concepts, this project demonstrates practical applications of SLAM, path planning, and sensor fusion.

**Features**
**_Smart Goal Planner_**

Frontier Detection Algorithm: Automatically identifies unexplored regions
Obstacle Density Analysis: Scores potential goals based on surrounding obstacles
Autonomous Exploration: Robot explores environments without manual intervention
Optimal Path Selection: Prioritizes safe, efficient navigation routes

**_Dynamic Obstacle Predictor_**

Motion Detection: Tracks moving obstacles using consecutive laser scans
Velocity Estimation: Calculates approaching object speeds
Emergency Stop System: Automatic braking when collision imminent (<0.3m)
Warning Levels: Three-tier alert system (INFO/WARNING/EMERGENCY)

** _Navigation Analytics Dashboard_**

Path Efficiency Metrics: Actual vs. optimal distance comparison
Real-time Performance: Speed, distance, and time tracking
Goal Achievement Rate: Success metrics and completion times
Data Visualization: Generate graphs for analysis and presentations


🏗️ **System Architecture**
┌─────────────────────────────────────────────────────────────┐
│                     Gazebo Simulation                        │
│                    (TurtleBot3 World)                        │
└────────────────────┬────────────────────────────────────────┘
                     │
        ┌────────────┼────────────┐
        │            │            │
        ▼            ▼            ▼
   ┌────────┐  ┌─────────┐  ┌──────────┐
   │ SLAM   │  │  Nav2   │  │ Sensors  │
   │Toolbox │  │ Stack   │  │ (LiDAR)  │
   └───┬────┘  └────┬────┘  └────┬─────┘
       │            │            │
       └────────────┼────────────┘
                    │
    ┌───────────────┼───────────────┐
    │               │               │
    ▼               ▼               ▼
┌─────────┐  ┌──────────┐  ┌──────────────┐
│ Smart   │  │Obstacle  │  │ Navigation   │
│ Goal    │  │Predictor │  │ Analytics    │
│ Planner │  │          │  │              │
└─────────┘  └──────────┘  └──────────────┘


🚀 Installation
**Option 1: Docker **
This method provides a pre-configured ROS2 environment - perfect if you're just starting! and since I'm using mac, docker makes wonders !!
bash
# 1.1 Pull and run ROS2 Docker container
docker run --platform linux/amd64 -p 6080:80 --shm-size=512m tiryoh/ros2-desktop-vnc:humble
# 1.2. Open browser and navigate to:
# http://localhost:6080
# Password: ubuntu

# 3. Open terminal in VNC viewer and proceed to "Setup TurtleBot3" below
bash# 1. Install ROS2 Humble
# I followed: https://docs.ros.org/en/humble/Installation.html

# 3. Install dependencies
sudo apt update
sudo apt install -y ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    python3-pip

# 4. Install Python packages
pip3 install numpy matplotlib scikit-learn
Setup TurtleBot3 (Both Methods)
bash# 1. Create workspace
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

# 5. Clone TurtleBot3 simulations
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# 6. Install dependencies
cd ~/turtlebot3_ws
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro humble

# 7. Build workspace
colcon build --symlink-install --might encounter warning/error issue but that should be fine

# 8. Source workspace and set environment variables
source ~/turtlebot3_ws/install/setup.bash
echo "source ~/turtlebot3_ws/install/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models" >> ~/.bashrc
source ~/.bashrc

** Usage**
**_Step 1: Create Map with SLAM_**
First, map the environment using Simultaneous Localization and Mapping.
**Terminal 1 - Launch Gazebo World:**
bashcd ~/turtlebot3_ws
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
**Terminal 2 - Launch SLAM:**
bashexport TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
**Terminal 3 - Teleoperation:**
bashexport TURTLEBOT3_MODEL=waffle_pi
ros2 run turtlebot3_teleop teleop_keyboard
📝 Drive the robot around using keyboard (w/a/s/d/x) for 3-5 minutes to map the entire world.
**Terminal 4 - Save Map:**
bashmkdir -p ~/maps
cd ~/maps
ros2 run nav2_map_server map_saver_cli -f turtlebot3_world_map
✅ Should now have turtlebot3_world_map.yaml and turtlebot3_world_map.pgm files!

**Step 2: Test Basic Navigation**
Before running enhanced features, verify basic navigation.
Terminal 1 - Gazebo:
bashros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
Terminal 2 - Navigation2:
bashexport TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/turtlebot3_world_map.yaml
In RViz:

Click "2D Pose Estimate" button
Click on map where robot is located and drag to set orientation
Click "Nav2 Goal" button
Click anywhere on the map - robot navigates autonomously!


**Step 3: Run Enhanced Navigation System**
Now launch all custom nodes for the full intelligent system.
Terminal 1 - Gazebo:
bashros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
Terminal 2 - Navigation2:
bashexport TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/turtlebot3_world_map.yaml
Terminal 3 - Smart Goal Planner:
bashros2 run autonomous_navigation_enhanced smart_goal_planner
This node automatically generates exploration goals every 10 seconds based on frontier detection and obstacle density analysis.
Terminal 4 - Obstacle Predictor:
bashros2 run autonomous_navigation_enhanced obstacle_predictor
Monitors laser scans for dynamic obstacles and publishes warnings/emergency stops.
Terminal 5 - Navigation Analytics:
bashros2 run autonomous_navigation_enhanced navigation_analytics
Tracks performance metrics and prints analytics every 5 seconds.

**Step 4: Monitor and Analyze**
