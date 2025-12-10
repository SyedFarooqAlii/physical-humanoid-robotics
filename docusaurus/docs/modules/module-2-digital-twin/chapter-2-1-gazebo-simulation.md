---
sidebar_position: 2
---

# Chapter 2-1: Gazebo Simulation

## Chapter Purpose (Engineering Intent)
This chapter establishes the foundation for creating realistic simulation environments using Gazebo Garden. Students will learn to model humanoid robots with accurate physics properties, implement sensor simulation, and create complex environments for testing robot behaviors before physical deployment.

## Systems & Subsystems Involved
- Gazebo simulation engine
- Robot model definition (URDF/SDF)
- Physics engine (ODE, Bullet, DART)
- Sensor simulation (cameras, lidar, IMU, force/torque)
- ROS 2 Gazebo plugins
- Environment/world modeling
- Collision detection and response systems

## Software Stack & Tools
- **Gazebo Garden** for simulation environment
- **ROS 2 Humble Hawksbill** for communication bridge
- **RViz2** for visualization and debugging
- **URDF/XACRO** for robot model definition
- **Gazebo ROS packages** for ROS 2 integration
- **SDFormat** for world and model description
- **Ignition libraries** for low-level simulation control

## Simulation vs Real-World Boundary
- **Simulation**: Physics-based modeling of robot dynamics, sensor responses, and environment interactions
- **Real-World**: Actual physical robot with real sensors and actuators
- **Boundary**: Simulation models validated against physical robot performance with appropriate fidelity metrics

## ROS 2 Interfaces
### Core Nodes
- `gazebo_ros_factory`: Dynamic model spawning and management
- `gazebo_ros_joint_state_publisher`: Joint state feedback from simulation
- `gazebo_ros_imu`: IMU sensor data publishing
- `gazebo_ros_camera`: Camera sensor data publishing
- `gazebo_ros_laser`: Laser scanner data publishing

### Topics
- `/joint_states` (sensor_msgs/msg/JointState): Robot joint positions and velocities
- `/imu/data` (sensor_msgs/msg/Imu): Inertial measurement unit data
- `/camera/image_raw` (sensor_msgs/msg/Image): Camera image data
- `/scan` (sensor_msgs/msg/LaserScan): LIDAR scan data
- `/gazebo/model_states` (gazebo_msgs/msg/ModelStates): Model position and state information

### Services
- `/gazebo/spawn_entity` (gazebo_msgs/srv/SpawnEntity): Spawn new models in simulation
- `/gazebo/delete_entity` (gazebo_msgs/srv/DeleteEntity): Remove models from simulation
- `/gazebo/reset_simulation` (std_srvs/srv/Empty): Reset entire simulation
- `/gazebo/set_model_state` (gazebo_msgs/srv/SetModelState): Set model position/orientation

### Actions
- `/gazebo/apply_body_wrench` (gazebo_msgs/action/ApplyBodyWrench): Apply forces/torques to bodies
- `/gazebo/set_link_state` (gazebo_msgs/action/SetLinkState): Control individual link states

## Perception / Planning / Control Responsibility
- **Perception**: Gazebo simulates sensor data for environment understanding
- **Planning**: Simulation provides environment models for path planning and motion planning
- **Control**: Joint controllers operate in simulation before deployment to real hardware

## Data Flow & Message Flow Description
1. Robot model loaded with URDF/SDF definition including physical properties
2. Gazebo physics engine calculates dynamics and kinematics
3. Sensor plugins generate synthetic sensor data (images, lidar, IMU, etc.)
4. ROS 2 bridge publishes sensor data to appropriate topics
5. Robot controllers subscribe to sensor data and publish commands
6. Joint command messages sent to simulated robot joints
7. Simulation updates robot state based on commands and physics
8. Loop continues in real-time or faster-than-real-time

## Hardware Dependency Level
- **Workstation**: Full simulation environment with GPU acceleration
- **Jetson Edge**: Light simulation for embedded validation
- **Physical Robot**: Reference for simulation model validation and tuning

## Failure Modes & Debug Surface
- **Physics Instability**: Joint limits, solver parameters, and mass distribution issues
- **Sensor Inaccuracy**: Simulation vs. real sensor differences
- **Timing Issues**: Simulation time vs. real-time synchronization problems
- **Debug Surface**: Gazebo GUI visualization, physics statistics, and ROS 2 introspection tools

## Capstone Mapping Tag
- **Simulation**: Complete Gazebo simulation environment for humanoid testing
- **Perception**: Sensor simulation for environment understanding
- **Navigation**: Simulated environment for path planning validation
- **Control**: Joint control validation in simulated physics environment