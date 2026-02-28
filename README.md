# Project Overview

This repository contains a ROS2-based autonomous mobile robot system developed using Gazebo simulation, LiDAR-based SLAM, and the Navigation2 (Nav2) stack. The project demonstrates a complete robotics autonomy pipeline from mapping and localization to path planning and waypoint-based navigation.

The system operates in two primary phases:

## 1. Mapping Phase (SLAM)

During the mapping phase, the robot is manually controlled using keyboard teleoperation (WASD). LiDAR scan data is processed by a SLAM algorithm to generate a 2D occupancy grid map of the environment. The generated map is saved as:

- `project.map.pgm`
- `project.map.yaml`

The occupancy grid uses:
- 0.05 meter resolution
- Trinary classification (free / occupied / unknown)
- Configurable occupancy thresholds

This map serves as the reference environment for autonomous navigation.

---

## 2. Navigation Phase (Nav2)

In the navigation phase, the ROS2 Navigation2 (Nav2) stack is launched with the saved map. The system performs:

- Localization using AMCL (particle filter)
- Global path planning
- Local trajectory control
- Obstacle avoidance
- Behavior tree-based mission execution

A custom ROS2 action client (`multi_nav_client.py`) sends waypoint goals using the `NavigateToPose` interface. The Navigation stack computes an optimal path and publishes velocity commands to drive the robot autonomously through the environment.

---

# System Architecture

The architecture is divided into three layers:

### Simulation Layer
Gazebo runs a custom world and spawns a TurtleBot3 robot. It publishes LiDAR scans, odometry, transforms, and simulation time.

### Navigation & Localization Layer
Nav2 loads the saved map, performs AMCL localization, computes global paths, and generates velocity commands using controller plugins.

### Application Layer
A custom ROS2 node sends sequential waypoint goals through the action interface, enabling mission-style navigation.

---

# Data Flow

LiDAR → SLAM → Occupancy Grid Map  
Map + Laser → AMCL → Pose Estimate  
Goal → Global Planner → Local Controller → `/cmd_vel` → Robot Motion  

---

# Technologies Used

- ROS2 (rclpy, launch system)
- Gazebo simulation
- TurtleBot3 platform
- LiDAR-based SLAM
- Navigation2 (Nav2)
- AMCL localization
- ROS2 Actions
- Occupancy grid mapping

---

# Key Engineering Concepts Demonstrated

- Distributed ROS2 node architecture
- Real-time sensor processing
- Particle filter localization
- Global and local path planning
- Behavior tree navigation
- Simulation-to-deployment workflow
- Autonomous waypoint execution

---

This project represents a complete perception-to-control robotics system built using ROS2 and modern navigation frameworks.
