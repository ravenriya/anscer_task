# Custom robot with Nav2

A ROS 2 based robot featuring navigation with optimized parameters for autonomous operation done using Zenoh middleware, can also be simulated using cycloneDDS, PLEASE NOTE THAT fastDDS makes the gazebo environment crash

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- ROS 2 Humble
- Gazebo Ignition Fortress
- RViz2
- Python 3.10+
- C++ 14 or newer

### Required Packages
```bash
# Install core dependencies
sudo apt update
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers
```

```bash
# Install Gazebo Ignition-Fortress
sudo apt-get install ignition-fortress
```

## System Components

### 1. Navigation System
The navigation system includes:
- Enhanced path planning algorithms using Nav2
- Optimized obstacle avoidance
- Real-time path optimization
- Parameter-tuned controllers

### 2. Simulation Environment
Fully integrated Gazebo simulation with:
- Warehouse environment
- Simulated sensors
- Performance monitoring
- Debug visualization

### 3. Localization System
Map-based positioning system using:
- AMCL localization
- Custom map integration
- Real-time position tracking

### 4. Trajectory Tracking System
Complete trajectory recording and visualization:
- Real-time trajectory recording (TrajectoryPublisher)
- Trajectory data saving to CSV files
- Trajectory visualization with markers
- Trajectory playback from saved files (TrajectoryReader)

## Running the Complete System

### 1. Start the Zenoh Daemon (if using Zenoh)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### 2. Launch Core System
```bash
# Terminal 1: Launch simulation and robot
ros2 launch anscer_task launch_sim.launch.py

<<<<<<< HEAD
# Terminal 2: Launch localization
ros2 launch anscer_task localization_launch.py map:=./src/anscer_task/maps/warehouse_save.yaml use_sim_time:=true
=======
# Terminal 2: Launch RViz2 with configuration
rviz2 -d src/anscer_task/config/view_bot.rviz
>>>>>>> 536814274f621dc302b256fd5922360fdc3340ae

# Terminal 3: Launch localization
ros2 launch anscer_task localization_launch.py map:=./src/anscer_task/maps/warehouse_save.yaml use_sim_time:=true

<<<<<<< HEAD
# Terminal 4: Run custom navigation node
ros2 run anscer_task navigation_node
=======
# Terminal 4: Launch navigation
ros2 launch anscer_task navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true
>>>>>>> 536814274f621dc302b256fd5922360fdc3340ae
```

### 3. Start Trajectory Recording
```bash
# Terminal 5: Run trajectory publisher node
ros2 run anscer_task trajectory_publisher #make sure you run this node while running the navigation stack and the fixed frame is set to map
```

### 4. Save Trajectory Data (when needed)
```bash
# Terminal 6: Save the trajectory data for the last 300 seconds to a file
ros2 service call /save_trajectory anscer_task/srv/SaveTrajectory "{filename: 'trajectory.csv', duration: 300.0}" #both file name and duration can be changed
```

### 5. Replay Saved Trajectory (optional)
```bash
# Terminal 6 or 7: Run trajectory reader node to visualize the saved trajectory
ros2 run anscer_task trajectory_reader #change the fixed frame to odom frame
```

## Launch Files Explained

### launch_sim.launch.py
Launches simulation components:
- Gazebo environment
- Robot spawning
- Sensor simulation
- Basic parameters

### localization_launch.py
Initiates localization:
- Map loading
- AMCL setup
- Transform configuration

### navigation_launch.py
Starts navigation stack:
- Parameter loading
- Path planning
- Obstacle avoidance

## Trajectory System

### Service Definition
The trajectory system uses a custom service to save trajectory data:
- **File location**: `srv/SaveTrajectory.srv`
- **Contents**:
  ```
  # Request
  string filename
  float32 duration  # Duration in seconds to save
  ---
  # Response
  bool success
  string message
  ```

### Trajectory System Nodes

### TrajectoryPublisher Node
Functionality:
- Records robot's path in real-time using TF transforms
- Visualizes the trajectory with red line markers in RViz
- Saves trajectory data to CSV files via service calls
- Configurable sampling rate (default: 10Hz)

Service Interface:
- `/save_trajectory` service (anscer_task/srv/SaveTrajectory)
- Parameters:
  - `filename`: Path to save the CSV file
  - `duration`: Optional time window for saving in seconds (300.0 = save last 5 minutes)
- Service Definition (srv/SaveTrajectory.srv):
  ```
  # Request
  string filename
  float32 duration  # Duration in seconds to save
  ---
  # Response
  bool success
  string message
  ```

### TrajectoryReader Node
Functionality:
- Loads saved trajectory data from CSV files
- Transforms and visualizes the trajectory in the odom frame
- Displays the trajectory with blue line markers in RViz
- Configurable publishing rate

Parameters:
- `trajectory_file`: Path to the CSV file (default: "trajectory.csv")
- `publish_rate`: Frequency of visualization updates in Hz (default: 1.0)

## Development Tools

### Building and Testing
```bash
# Build specific packages
<<<<<<< HEAD
colcon build --packages-select anscer_task or colcon build --symlink-install
=======
colcon build --packages-select anscer_task

# Build with symlink install
colcon build --symlink-install
```
>>>>>>> 536814274f621dc302b256fd5922360fdc3340ae

## Common Issues and Solutions

### Navigation Issues
1. **Map Not Loading**
   - Verify map path
   - Check file permissions
   - Confirm YAML format

2. **Navigation Failures**
   - Check transform tree
   - Verify localization
   - Review costmap parameters

3. **Simulation Issues**
   - Confirm use_sim_time settings
   - Check Gazebo launch
   - Verify sensor data

### Trajectory System Issues
1. **Trajectory Not Visualizing**
   - Check TF transformations (map → odom → base_link)
   - Verify RViz is subscribed to the marker topics
   - Confirm marker scale and color settings

2. **CSV File Issues**
   - Ensure write permissions for the save directory
   - Check CSV format compatibility
   - Verify timestamp format in saved files
