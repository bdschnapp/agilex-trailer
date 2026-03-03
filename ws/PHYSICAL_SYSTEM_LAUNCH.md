# Physical System Launch Files

This document describes the new launch files for running the planning system on the physical Hunter vehicle.

## Overview

Two new launch files have been created to run the full planning and control stack on the physical vehicle:

1. **`vehicle.launch.py`** - Physical vehicle without trailer
2. **`vehicle_trailer.launch.py`** - Physical vehicle with trailer

These launch files replace Gazebo simulation with real hardware:
- **Localization**: `lidarslam` package (instead of Gazebo odometry)
- **Control**: `canbridge` package via CAN bus (instead of Gazebo ros2_control)

## Architecture Comparison

### Simulation (simulation.launch.py)
```
Gazebo → ros2_control → /ackermann_like_controller/odom
                     ← /ackermann_like_controller/cmd_vel
```

### Physical System (vehicle.launch.py / vehicle_trailer.launch.py)
```
RSLidar → lidarslam → /current_pose → pose_to_odom → /odom
Planner → /ackermann_cmd → ackermann_to_can → /joy → canbridge → CAN bus → Vehicle
```

## Key Components

### 1. Localization Stack (lidarslam)
- **scanmatcher_node**: Performs lidar-based odometry using NDT scan matching
  - Input: `/rslidar_points` (PointCloud2)
  - Output: `/current_pose` (PoseStamped)
  - Publishes TF: `map` → `base_link`

- **graph_based_slam_node**: Performs loop closure for long-term consistency
  - Uses graph optimization to correct drift
  - Saves/loads maps for persistent localization

### 2. Odometry Conversion
- **pose_to_odom**: Converts `/current_pose` (PoseStamped) to `/odom` (Odometry)
  - Required because planning nodes expect Odometry messages
  - Sets covariance and twist fields appropriately

- **trailer_odom_builder** (trailer version only): Computes trailer odometry
  - Combines tractor odometry with trailer joint angle
  - Publishes `/trailer_odom` for planning

### 3. Control Stack
- **ackermann_to_can**: Converts planning commands to vehicle control
  - Input: `/ackermann_cmd` or `/trailer_cmd` (AckermannDrive)
  - Output: `/joy` (Joy message)
  - Includes safety limits (max speed, max steering angle)

- **canbridge**: Sends control commands to vehicle via CAN
  - Input: `/joy` (from manual joystick or ackermann_to_can)
  - Output: CAN frames to vehicle hardware
  - Publishes: `/vehicle/VelocityReport`, `/vehicle/SteeringReport`

### 4. Planning & Control (same as simulation)
- **planner_node**: Hybrid A* path planning
- **mpc_node**: Model Predictive Control for trajectory tracking
- **footprint_publisher**: Vehicle footprint visualization

### 5. Sensors (sensors.launch.py)
- **RSLidar**: 3D lidar for mapping and localization
- **IMU**: Witmotion IMU (optional, currently not fused)
- **GPS**: uBlox ZED-F9P (optional, currently not used)
- **CAN**: Vehicle control and feedback

## New Files Created

### Launch Files
1. `/src/hunter_launch/launch/vehicle.launch.py` - Physical vehicle launch (no trailer)
2. `/src/hunter_launch/launch/vehicle_trailer.launch.py` - Physical vehicle launch (with trailer)

### Python Nodes
3. `/src/planning/planning/pose_to_odom.py` - PoseStamped → Odometry converter
4. `/src/planning/planning/ackermann_to_can.py` - AckermannDrive → Joy converter

### Updated Files
5. `/src/planning/setup.py` - Added new executables to entry_points

## Usage

### 1. Build the workspace
```bash
cd /home/ben/Ben/Electrans/planning_sim_env/ws
colcon build --packages-select planning hunter_launch
source install/setup.bash
```

### 2. Launch the physical system

**Without trailer:**
```bash
ros2 launch hunter_launch vehicle.launch.py
```

**With trailer:**
```bash
ros2 launch hunter_launch vehicle_trailer.launch.py
```

### 3. Optional arguments

**Disable planning (sensors and localization only):**
```bash
ros2 launch hunter_launch vehicle.launch.py use_planning:=false
```

**Disable specific sensors:**
```bash
ros2 launch hunter_launch vehicle.launch.py \
    enable_lidar:=false \
    enable_imu:=false \
    enable_gps:=false
```

**Use virtual CAN for testing:**
```bash
ros2 launch hunter_launch vehicle.launch.py can_if:=vcan0
```

**Change ROS domain:**
```bash
ros2 launch hunter_launch vehicle.launch.py ros_domain_id:=42
```

**Disable RViz:**
```bash
ros2 launch hunter_launch vehicle.launch.py enable_rviz:=false
```

## Important Notes

### Before First Run

1. **Build the planning package** to install new executables:
   ```bash
   colcon build --packages-select planning
   ```

2. **Check lidarslam configuration**:
   - Verify parameters in `/src/lidarSlam/src/lidarslam_ros2/lidarslam/param/lidarslam.yaml`
   - Adjust initial pose if needed
   - Set `use_imu: true` if you want IMU integration

3. **Prepare a PCD map**:
   - Create a map using lidarslam in mapping mode first
   - Save the map to `/src/hunter_launch/maps/map.pcd`
   - Or disable map nodes if doing SLAM without a prior map

4. **CAN interface setup**:
   - Ensure CAN interface is properly configured
   - The launch file will attempt to bring up the interface automatically
   - If using hardware, ensure `can0` exists: `ip link show can0`

5. **Safety considerations**:
   - Start with `max_speed` set low (currently 0.5 m/s)
   - Test with joystick control first before enabling autonomous planning
   - Keep emergency stop accessible

### Calibration and Tuning

**TF Tree calibration:**
- Verify `base_link → rsLidar` transform is accurate
- Currently set to identity (0,0,0,0,0,0,1)
- Adjust in vehicle.launch.py line 161 if needed

**Lidarslam tuning:**
- `ndt_resolution`: Affects matching accuracy (default: 2.0)
- `scan_min_range`, `scan_max_range`: Filter lidar range
- `use_imu`, `use_odom`: Enable sensor fusion if sensors are calibrated

**Control tuning:**
- `max_speed` in ackermann_to_can: Safety speed limit
- `max_steering_angle`: Should match vehicle hardware limits
- Adjust wheelbase parameter if needed (currently 0.65m)

**Planning parameters:**
- Grid map, hybrid A*, optimizer, trailer, controller params
- Located in `/src/planner_ros2/params/*.yaml`

## Troubleshooting

### No odometry messages
- Check if scanmatcher is publishing: `ros2 topic echo /current_pose`
- Verify lidar data: `ros2 topic echo /rslidar_points`
- Check TF tree: `ros2 run tf2_tools view_frames`

### Vehicle not responding to commands
- Verify CAN interface is up: `ip link show can0`
- Check canbridge is receiving Joy messages: `ros2 topic echo /joy`
- Monitor CAN traffic: `candump can0`
- Check vehicle feedback: `ros2 topic echo /vehicle/VelocityReport`

### Planning failures
- Verify occupancy grid is published: `ros2 topic echo /occupancy_grid`
- Check if map is loaded: `ros2 topic echo /global_map`
- Ensure odometry is available: `ros2 topic echo /odom` or `/trailer_odom`

### TF errors
- Check TF tree: `ros2 run tf2_tools view_frames`
- Expected tree: `world → map → base_link` (or `map → odom → base_link`)
- Lidarslam should publish `map → base_link`
- For trailer: also `base_link → trailer_base_link`

### Performance issues
- Reduce lidar point density or increase `vg_size_for_input`
- Disable graph-based SLAM if loop closure is not needed
- Lower planning frequency if CPU is overloaded

## Next Steps

### Required Actions

1. **Test and tune lidarslam**:
   - Create a map of your environment
   - Tune NDT parameters for your lidar
   - Verify localization accuracy

2. **Calibrate vehicle parameters**:
   - Measure actual wheelbase
   - Determine safe max speed and steering angle
   - Test control response and adjust gains if needed

3. **Create/update PCD map**:
   - Record a bag file while manually driving
   - Use lidarslam to generate a map
   - Place map.pcd in the maps directory

4. **Test control pipeline**:
   - Start with joystick control (set `use_planning:=false`)
   - Verify vehicle responds correctly
   - Then enable autonomous planning

### Optional Enhancements

1. **IMU integration**:
   - Calibrate IMU to lidar frame
   - Enable `use_imu: true` in lidarslam params
   - May improve localization in dynamic environments

2. **GPS integration**:
   - Use GPS for global localization
   - Implement GPS-based map initialization
   - Useful for large-scale navigation

3. **Sensor fusion**:
   - Fuse lidarslam with wheel odometry
   - Use Extended Kalman Filter (EKF)
   - Provides more robust state estimation

4. **Dynamic reconfiguration**:
   - Add parameter services for runtime tuning
   - Create safety monitoring nodes
   - Implement watchdog for autonomous operation

5. **Trailer angle sensing**:
   - Currently uses lidar-based hitch angle detection
   - May want to add dedicated angle sensor
   - Update trailer_odom_builder parameters accordingly

## Topic Reference

### Key Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/rslidar_points` | PointCloud2 | Lidar point cloud |
| `/current_pose` | PoseStamped | Vehicle pose from lidarslam |
| `/odom` | Odometry | Converted odometry for planning |
| `/trailer_odom` | Odometry | Trailer odometry (trailer version) |
| `/ackermann_cmd` | AckermannDrive | Control command (no trailer) |
| `/trailer_cmd` | AckermannDrive | Control command (with trailer) |
| `/joy` | Joy | Joystick/converted control |
| `/CAN/can0/receive` | Frame | CAN bus incoming |
| `/CAN/can0/transmit` | Frame | CAN bus outgoing |
| `/vehicle/VelocityReport` | VelocityReport | Vehicle speed feedback |
| `/vehicle/SteeringReport` | SteeringReport | Vehicle steering feedback |
| `/global_map` | PointCloud2 | Filtered map |
| `/occupancy_grid` | OccupancyGrid | 2D grid for planning |

### TF Frames

| Frame | Parent | Publisher |
|-------|--------|-----------|
| `world` | - | Static |
| `map` | `world` | Static |
| `base_link` | `map` | lidarslam |
| `rsLidar` | `base_link` | Static |
| `trailer_base_link` | `base_link` | joint_state_publisher |

## Contact

For issues or questions:
- Check logs: `ros2 launch hunter_launch vehicle.launch.py 2>&1 | tee launch.log`
- Monitor system: `ros2 node list`, `ros2 topic list`
- Debug TF: `ros2 run tf2_tools view_frames`
- Check dependencies: `rosdep check --from-paths src`
