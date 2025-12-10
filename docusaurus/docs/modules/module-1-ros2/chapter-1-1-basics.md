---
sidebar_position: 3
---

# Chapter 1-1: ROS 2 Basics

## Chapter Purpose (Engineering Intent)
This chapter introduces students to the fundamental concepts of ROS 2, providing the essential knowledge needed to understand and work with the Robot Operating System 2 framework. Students will learn the basic architecture, installation, and core concepts that form the foundation for all subsequent ROS 2 development.

## Systems & Subsystems Involved
- ROS 2 core runtime system
- DDS (Data Distribution Service) middleware
- RMW (ROS Middleware) layer
- Node execution manager
- Parameter server
- Clock and time management system

## Software Stack & Tools
- ROS 2 Humble Hawksbill (LTS version)
- Fast DDS or Cyclone DDS middleware
- Python 3.8+ or C++17 development tools
- colcon build system
- rosdep dependency manager
- rclpy/rclcpp client libraries
- ROS 2 command-line tools (ros2 cli)

## Simulation vs Real-World Boundary
- **Simulation**: Initial learning and testing using ROS 2 in Docker containers or on workstations
- **Real-World**: Deployment to robot hardware with appropriate safety considerations
- **Boundary**: Students first learn concepts in simulation, then apply to real hardware

## ROS 2 Interfaces
### Nodes
- `talker`: Simple publisher node (example from tutorials)
- `listener`: Simple subscriber node (example from tutorials)
- `parameter_node`: Node demonstrating parameter usage

### Topics
- `/chatter` (std_msgs/msg/String): Basic string message topic for learning
- `/rosout` (rcl_interfaces/msg/Log): System logging topic

### Services
- `std_srvs/srv/Empty`: Basic service example for learning

### Actions
- Not covered in basics chapter

## Perception / Planning / Control Responsibility
- **Perception**: Basic sensor data handling concepts
- **Planning**: Not applicable in basics
- **Control**: Basic command execution concepts

## Data Flow & Message Flow Description
1. ROS 2 daemon starts and manages the distributed system
2. Nodes are launched and register with the system
3. Publishers send messages to topics
4. Subscribers receive messages from topics
5. Services handle request/response communication
6. Parameters are shared across nodes

## Hardware Dependency Level
- **Workstation**: Primary learning environment
- **Jetson Edge**: Not applicable for basics
- **Physical Robot**: Not applicable for basics

## Failure Modes & Debug Surface
- **Network issues**: DDS communication failures
- **Node crashes**: Individual node failure handling
- **Message loss**: Quality of service configuration
- **Debug surface**: ros2 topic, ros2 service, ros2 node, ros2 param tools

## Capstone Mapping Tag
- **Control**: Foundation for all control communication
- **Navigation**: Required for navigation system communication