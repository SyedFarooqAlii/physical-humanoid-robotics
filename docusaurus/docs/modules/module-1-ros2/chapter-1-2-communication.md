---
sidebar_position: 4
---

# Chapter 1-2: ROS 2 Communication Patterns

## Chapter Purpose (Engineering Intent)
This chapter explores the various communication patterns available in ROS 2, including topics, services, and actions. Students will learn when to use each pattern and how to implement them effectively in robotic applications, with specific focus on humanoid robot communication requirements.

## Systems & Subsystems Involved
- Publisher-subscriber communication system
- Request-response service system
- Goal-oriented action system
- Quality of Service (QoS) configuration
- Message serialization and transport
- Network communication layer

## Software Stack & Tools
- ROS 2 Humble Hawksbill
- rclpy/rclcpp client libraries
- Custom message/service/action definitions
- QoS policy configuration tools
- Network monitoring tools (netstat, iftop)
- ROS 2 introspection tools (ros2 topic, ros2 service, ros2 action)

## Simulation vs Real-World Boundary
- **Simulation**: Communication patterns tested with simulated robot components
- **Real-World**: Communication patterns validated with actual robot hardware
- **Boundary**: QoS settings may need adjustment for real-world network conditions

## ROS 2 Interfaces
### Nodes
- `sensor_publisher`: Publishes sensor data using pub/sub pattern
- `command_service`: Handles command requests using service pattern
- `navigation_action_server`: Executes navigation goals using action pattern

### Topics
- `/sensor_data` (sensor_msgs/msg/JointState): Joint state sensor data
- `/robot_status` (std_msgs/msg/String): Robot operational status
- `/command_feedback` (std_msgs/msg/Float64): Command execution feedback

### Services
- `/execute_command` (custom service): Execute specific robot commands
- `/get_robot_state` (custom service): Retrieve current robot state

### Actions
- `/move_to_goal` (geometry_msgs/action/MoveAction): Move robot to specific goal
- `/grasp_object` (custom action): Execute object grasping sequence

## Perception / Planning / Control Responsibility
- **Perception**: Sensor data publishing and sharing
- **Planning**: Requesting path planning services
- **Control**: Executing action-based control commands

## Data Flow & Message Flow Description
1. Sensor nodes publish data to appropriate topics
2. Perception nodes subscribe to sensor data and process
3. Planning nodes receive processed data and generate plans
4. Control nodes execute plans and provide feedback
5. Service calls are made for synchronous operations
6. Actions are used for long-running, goal-oriented tasks

## Hardware Dependency Level
- **Workstation**: Development and simulation testing
- **Jetson Edge**: Deployment with optimized communication for edge constraints
- **Physical Robot**: Real-time communication with hardware interfaces

## Failure Modes & Debug Surface
- **Message drops**: QoS configuration and network issues
- **Service timeouts**: Unresponsive service providers
- **Action preemption**: Goal cancellation during execution
- **Debug surface**: rqt_graph, ros2 doctor, network analysis tools

## Capstone Mapping Tag
- **Navigation**: Essential for navigation communication
- **Planning**: Required for planning system communication
- **Control**: Foundation for control system communication