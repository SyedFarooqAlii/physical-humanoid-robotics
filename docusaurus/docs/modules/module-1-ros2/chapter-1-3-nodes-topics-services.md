---
sidebar_position: 5
---

# Chapter 1-3: Nodes, Topics, Services

## Chapter Purpose (Engineering Intent)
This chapter provides in-depth coverage of the three fundamental communication primitives in ROS 2: nodes, topics, and services. Students will learn to design and implement these components for humanoid robot applications, understanding their appropriate use cases and implementation patterns.

## Systems & Subsystems Involved
- Node lifecycle management system
- Topic publishing/subscribing infrastructure
- Service request/response handling
- Parameter management system
- Logging and diagnostic systems
- Node introspection and monitoring

## Software Stack & Tools
- ROS 2 Humble Hawksbill with rclpy/rclcpp
- Custom message definitions using .msg files
- Service definitions using .srv files
- colcon build system for compilation
- launch files for node orchestration
- rqt tools for visualization
- ros2 command-line tools for management

## Simulation vs Real-World Boundary
- **Simulation**: Nodes tested with simulated sensors and actuators
- **Real-World**: Nodes connected to actual robot hardware with safety considerations
- **Boundary**: Parameter configurations and safety checks differ between environments

## ROS 2 Interfaces
### Nodes
- `imu_sensor_node`: Publishes IMU sensor data
- `motor_control_node`: Subscribes to motor commands and controls hardware
- `health_monitor_node`: Monitors system health and publishes status
- `command_service_node`: Provides services for robot commands

### Topics
- `/imu/data` (sensor_msgs/msg/Imu): IMU sensor readings
- `/joint_commands` (std_msgs/msg/Float64MultiArray): Joint position commands
- `/robot_health` (std_msgs/msg/String): System health status
- `/tf` (tf2_msgs/msg/TFMessage): Transform data for coordinate frames

### Services
- `/reset_robot` (std_srvs/srv/Empty): Reset robot to safe state
- `/get_joint_positions` (custom service): Retrieve current joint positions
- `/set_robot_mode` (custom service): Change robot operational mode
- `/emergency_stop` (std_srvs/srv/Trigger): Emergency stop service

## Perception / Planning / Control Responsibility
- **Perception**: Sensor nodes publish data for perception algorithms
- **Planning**: Service interfaces for requesting planning operations
- **Control**: Command nodes execute control algorithms and send to hardware

## Data Flow & Message Flow Description
1. Sensor nodes continuously publish sensor data to topics
2. Perception nodes subscribe to sensor data and publish processed information
3. Planning nodes subscribe to perception data and provide planning services
4. Control nodes subscribe to planning outputs and publish motor commands
5. Service calls handle synchronous operations like mode changes
6. Parameter server provides configuration data to all nodes

## Hardware Dependency Level
- **Workstation**: Development and simulation environment
- **Jetson Edge**: Optimized for embedded system constraints
- **Physical Robot**: Direct interface with robot hardware and safety systems

## Failure Modes & Debug Surface
- **Node crashes**: Process failures and restart mechanisms
- **Topic overflow**: Message queue limitations and QoS settings
- **Service timeouts**: Unresponsive service requests
- **Debug surface**: rqt_graph, ros2 node, ros2 topic, ros2 service tools

## Capstone Mapping Tag
- **Control**: Core component for robot control communication
- **Navigation**: Required for navigation system integration
- **Planning**: Foundation for planning system interfaces