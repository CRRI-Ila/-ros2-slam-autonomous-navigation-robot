## Setup and Execution – Project_Pkg

---

### Step 1: Launch Gazebo World

Command:

ros2 launch Project_Pkg Project.launch.py


This launches:
- Gazebo server and client  
- Custom world file (`Project.world`)  
- TurtleBot3 robot  
- Simulation time  

---

### Step 2: Run Cartographer SLAM (Mapping Mode)

Command:

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True


This starts LiDAR-based SLAM to generate a 2D occupancy grid map while the robot explores.

---

### Step 3: Teleoperate the Robot

Command:

ros2 run teleop_twist_keyboard teleop_twist_keyboard


Use the keyboard (WASD keys) to drive the robot and allow SLAM to build the environment map.

---

### Step 4: Save the Map

Create a maps directory:

mkdir -p ~/ros2_ws/src/Project_Pkg/maps


Navigate to the maps directory:

cd ~/ros2_ws/src/Project_Pkg/maps


Save the map:

ros2 run nav2_map_server map_saver_cli -f project.map


This generates:
- `project.map.pgm`
- `project.map.yaml`

---

### Step 5: Launch Navigation Stack (Nav2)

Command:

ros2 launch Project_Pkg nav2.launch.py


This loads the saved map and starts:
- Map server  
- AMCL localization  
- Global planner  
- Local controller  
- Behavior tree navigator  
- RViz  

---

### Step 6: Run Waypoint Navigation Client

Command:

ros2 run Project_Pkg multi_nav_client


This sends waypoint goals using the `NavigateToPose` action interface and allows the robot to navigate autonomously.

---

## System Workflow Summary

1. Launch simulation environment  
2. Spawn robot  
3. Build map using Cartographer SLAM  
4. Save occupancy grid map  
5. Load map into Nav2  
6. Execute autonomous waypoint navigation  

---
