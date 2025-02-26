# Custom robot with Nav2

A ROS 2 based robot featuring custom navigation nodes and optimized parameters for autonomous operation done using Zenoh middleware, can also be simulated using cycloneDDS, PLEASE NOTE THAT fastDDS makes the gazebo environment crash


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
    ros-humble-ros2-controllers \
```
# Install Gazebo Ignition-Fortress
sudo apt-get install ignition-fortress


## System Components

### 1. Custom Navigation Node
The specialized navigation implementation includes:
- Enhanced path planning algorithms
- Custom cost functions
- Optimized obstacle avoidance
- Real-time path optimization

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

### 3. Navigation System using Nav2
Map-based navigation system using:
- AMCL localization
- Map integration
- Real-time dynamic obstacle avoidance

## Running the Complete System

### 1. Start the Zenoh Daemon (if using Zenoh)
```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
```

### 2. Launch Core System
```bash
# Terminal 1: Launch simulation and robot
ros2 launch accl_task launch_sim.launch.py

# Terminal 2: Launch localization
ros2 launch accl_task localization_launch.py map:=./src/accl_task/maps/warehouse_save.yaml use_sim_time:=true

# Terminal 3: Launch custom navigation
ros2 launch my_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true

# Terminal 4: Run custom navigation node
ros2 run accl_task navigation_node
```
## Launch Files and Navigation Node Explained

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
- Custom navigation node
- Parameter loading
- Path planning
- Obstacle avoidance

### navigation_node.cpp
- Interfaces with running AMCL node and sets initial pose
- Interfaces with Running Nav2 nodes and gives coordinates for the robot to traverse through
- Adds visual markers after reaching a goal

## Configuration Files

### nav_params.yaml
Optimized Configuration
The optimized configuration uses MPPI controller (nav2_mppi_controller::MPPIController), whereas the default uses the DWB controller (dwb_core::DWBLocalPlanner). The MPPI controller implements Model Predictive Path Integral control, which provides more advanced trajectory planning and obstacle avoidance capabilities.
The optimized configuration includes both precise_goal_checker and general_goal_checker, while the default only has general_goal_checker. The precise goal checker has tighter tolerances (0.1m for both position and orientation) compared to the default's 0.25m tolerances, allowing for more accurate positioning at the destination.
MPPI Controller Specific Parameters
The optimized configuration includes numerous MPPI-specific parameters that significantly enhance navigation performance:

Time steps of 56 and model time step of 0.05 seconds for trajectory prediction
Large batch size of 2000 for thorough trajectory evaluation
Appropriate noise parameters for trajectory sampling
Higher maximum velocity (0.5 m/s compared to default 0.26 m/s)
Well-tuned acceleration limits for smoother motion
Optimization parameters like temperature (0.3) and gamma (0.015)
Specialized critics including ConstraintCritic, CostCritic, GoalCritic, GoalAngleCritic, PathAlignCritic, PathFollowCritic, PathAngleCritic, and PreferForwardCritic
High collision cost (1,000,000) for safer navigation

Planner Server
A key difference in the optimized configuration is enabling A* algorithm for path planning (use_astar: true), whereas the default uses Dijkstra's algorithm (use_astar: false). A* can provide more efficient path planning in complex environments by using heuristics to guide the search, potentially resulting in faster path computation and better routes.

Key Advantages of the Optimized Configuration
More intelligent path planning with A* algorithm, which can find optimal paths more efficiently than Dijkstra's algorithm in complex environments
Advanced controller implementation using MPPI, which provides better handling of dynamic obstacles, more sophisticated trajectory generation, and smoother motion planning through its predictive capabilities
Higher performance capabilities with faster maximum velocity (0.5 m/s vs 0.26 m/s) and better acceleration handling
More precise final positioning with tighter goal tolerances and balanced approach behavior
Enhanced path following behavior with specialized critics that maintain a better balance between staying on the planned path and adapting to environmental changes
Improved safety through higher collision costs and more sophisticated obstacle avoidance through the predictive capabilities of MPPI
Better computational efficiency through batch processing for trajectory evaluation, appropriate pruning distance, and efficient update parameters
Fine tuned to match environmental constraints and robot capabilities

## Development Tools

### Building and Testing
```bash
# Build specific packages
colcon build --packages-select accl_task or colcon build --symlink-install

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
