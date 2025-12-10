---
sidebar_position: 6
---

# Chapter 1-4: Actions and Advanced Interfaces

## Chapter Purpose (Engineering Intent)
This chapter covers advanced ROS 2 communication patterns, focusing on actions for long-running, goal-oriented tasks. Students will learn to implement action servers and clients for humanoid robot behaviors, as well as advanced interfaces like parameters, logging, and diagnostics.

## Systems & Subsystems Involved
- Action server/client infrastructure
- Parameter management system
- Logging and diagnostic framework
- Lifecycle node management
- Composition and node grouping
- Time and clock synchronization

## Software Stack & Tools
- ROS 2 Humble Hawksbill with rclpy/rclcpp
- Action definition files (.action)
- Lifecycle nodes and composition
- Parameters and configuration management
- Logging (rcl_logging) and diagnostics
- Timers and callbacks
- Client libraries with async support

## Simulation vs Real-World Boundary
- **Simulation**: Actions tested with simulated robot behaviors
- **Real-World**: Actions connected to actual robot behaviors with safety protocols
- **Boundary**: Action feedback and result handling may differ based on real-world constraints

## ROS 2 Interfaces
### Nodes
- `navigation_action_server`: Handles navigation goals with feedback
- `manipulation_action_server`: Executes manipulation tasks with progress
- `calibration_action_server`: Performs sensor calibration procedures
- `lifecycle_manager_node`: Manages node lifecycle states

### Actions
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose): Navigate to specified pose
- `/pick_object` (custom action): Pick up objects with feedback
- `/calibrate_sensors` (custom action): Calibrate sensor systems
- `/execute_trajectory` (control_msgs/action/FollowJointTrajectory): Execute joint trajectories

### Topics
- `/action_feedback` (various): Action feedback messages
- `/action_status` (action_msgs/msg/GoalStatusArray): Action status updates
- `/diagnostics` (diagnostic_msgs/msg/DiagnosticArray): System diagnostics

### Services
- `/change_state` (lifecycle_msgs/srv/ChangeState): Lifecycle management
- `/get_parameters` (rcl_interfaces/srv/GetParameters): Parameter retrieval

## Perception / Planning / Control Responsibility
- **Perception**: Action-based sensor calibration and configuration
- **Planning**: Action-based path planning with progress feedback
- **Control**: Action-based trajectory execution and manipulation

## Data Flow & Message Flow Description
1. Action clients send goals to action servers
2. Action servers provide feedback during execution
3. Action servers return results upon completion
4. Clients can preempt goals during execution
5. Parameter updates propagate to all relevant nodes
6. Diagnostic information is aggregated and reported

## Hardware Dependency Level
- **Workstation**: Development and testing environment
- **Jetson Edge**: Execution with resource constraints
- **Physical Robot**: Safety-critical action execution with hardware interfaces

## Failure Modes & Debug Surface
- **Action timeouts**: Long-running operations that exceed time limits
- **Goal preemption**: Actions cancelled during execution
- **Resource exhaustion**: Memory/CPU limits during action execution
- **Debug surface**: rqt_plot, ros2 action, ros2 lifecycle, diagnostic tools

## Capstone Mapping Tag
- **Navigation**: Core component for navigation system
- **Manipulation**: Essential for object manipulation
- **Planning**: Required for complex task planning
- **Control**: Foundation for trajectory execution