# ROS2 Autonomous Mobile Robot  
## Cartographer SLAM + Nav2 Navigation in Gazebo

---

# Project Overview

This project implements a complete autonomous robotics pipeline using ROS2, Gazebo, Cartographer SLAM, and the Navigation2 (Nav2) stack.

The system allows a mobile robot to:

- Operate inside a custom Gazebo environment
- Generate a 2D occupancy grid map using LiDAR (Cartographer)
- Save and reload the generated map
- Localize within the map
- Navigate autonomously using waypoint goals
- Continuously update the map when operating in SLAM mode

This project demonstrates perception, mapping, localization, planning, and control in a distributed ROS2 architecture.

---

# System Workflow

## Phase 1 — Environment Setup

A custom Gazebo world is launched using:


ros2 launch Project_Pkg Project.launch.py


This:

- Starts Gazebo server and client
- Spawns TurtleBot3
- Enables simulation time
- Publishes sensor data and transforms

Gazebo provides:

- `/scan` (LiDAR)
- `/odom` (odometry)
- `/tf` (transforms)
- `/clock` (simulation time)

---

## Phase 2 — Mapping Mode (Cartographer SLAM)

In mapping mode, Cartographer processes LiDAR scan data to build a real-time occupancy grid map.

The robot is manually controlled using teleoperation while exploring the environment.

Data flow:

LiDAR → Cartographer → Occupancy Grid Map

The resulting map is saved as:


project.map.pgm
project.map.yaml


Map properties:

- Resolution: 0.05 meters per pixel
- Trinary occupancy representation
- Configurable free/occupied thresholds

This map becomes the reference for navigation.

---

## Phase 3 — Navigation Mode (Nav2)

After mapping, the Navigation2 stack is launched:


ros2 launch Project_Pkg nav2.launch.py


Nav2 loads the saved map and starts:

- Map Server
- AMCL localization
- Global planner
- Local controller
- Behavior tree navigator
- Costmaps
- RViz visualization

Localization is performed using AMCL:

Laser + Map → Particle Filter → Pose Estimate

---

## Phase 4 — Waypoint Navigation

A custom ROS2 node (`multi_nav_client.py`) sends waypoint goals using the `NavigateToPose` action interface.

Navigation flow:

Waypoint Goal  
→ Nav2 Global Planner  
→ Local Controller  
→ Velocity Commands  
→ Robot Motion  

The robot autonomously moves between defined map coordinates.

---

# Optional Continuous SLAM Mode

If Cartographer remains active during navigation, the map can continue updating in real time as the robot explores new regions or environmental changes occur.

This enables dynamic mapping and adaptive navigation.

---

# System Architecture

The system consists of three logical layers:

### 1. Simulation Layer (Gazebo)
Provides physics simulation, robot model, and sensor data.

### 2. Mapping & Localization Layer
- Cartographer (SLAM mode)
- Map Server (static map mode)
- AMCL localization

### 3. Navigation Layer
- Global path planning
- Local trajectory control
- Obstacle avoidance
- Behavior tree execution
- ROS2 action-based waypoint interface

---

# Data Flow Overview

LiDAR  
→ Cartographer (Mapping Mode)  
→ Occupancy Grid  

Map + Laser  
→ AMCL  
→ Robot Pose  

Goal  
→ Nav2 Planner  
→ Controller  
→ `/cmd_vel`  
→ Robot Motion  

---

# Technologies Used

- ROS2 (rclpy)
- Gazebo
- TurtleBot3
- Cartographer SLAM
- Navigation2 (Nav2)
- AMCL
- Occupancy grid mapping
- ROS2 Actions
- Behavior Trees

---

# Engineering Concepts Demonstrated

- SLAM-based mapping
- Occupancy grid representation
- Monte Carlo localization
- Global and local path planning
- Real-time distributed node architecture
- ROS2 action client-server communication
- Simulation-to-autonomy workflow
- Modular robotics system design

---

# How to Run

### 1. Launch Simulation

ros2 launch Project_Pkg Project.launch.py


### 2. Run Cartographer SLAM (Mapping Mode)

ros2 launch <cartographer_launch>


### 3. Save Map

ros2 run nav2_map_server map_saver_cli -f project.map


### 4. Launch Navigation

ros2 launch Project_Pkg nav2.launch.py


### 5. Run Waypoint Client

ros2 run Project_Pkg multi_nav_client


---

# Outcome

The robot successfully:

- Generated a LiDAR-based occupancy grid map
- Localized within the saved map
- Planned collision-free paths
- Executed autonomous waypoint navigation
- Supported both static map navigation and continuous SLAM updates

This project represents a complete robotics autonomy pipeline from environment generation to autonomous motion control.

