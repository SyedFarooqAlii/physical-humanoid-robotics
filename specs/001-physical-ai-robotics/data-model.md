# Data Model: Physical AI & Humanoid Robotics

## Entity: Chapter Specification
- **Purpose**: Technical specification for individual book chapters
- **Fields**:
  - chapterId: string (unique identifier)
  - moduleId: string (module identifier)
  - title: string (chapter title)
  - purpose: string (engineering intent)
  - systems: [string] (systems and subsystems involved)
  - softwareStack: [string] (software tools and frameworks)
  - simulationBoundary: string (simulation vs real-world distinction)
  - ros2Interfaces: [Ros2Interface] (ROS 2 communication elements)
  - responsibilities: [string] (perception/planning/control allocation)
  - dataFlow: string (data and message flow description)
  - hardwareLevel: enum (Workstation | Jetson Edge | Physical Robot)
  - failureModes: [string] (potential failure modes)
  - capstoneTags: [CapstoneCapability] (capstone mapping tags)

## Entity: Module
- **Purpose**: Collection of related chapters around a core robotics concept
- **Fields**:
  - moduleId: string (unique module identifier)
  - title: string (module title)
  - description: string (module overview)
  - chapters: [ChapterSpecification] (chapters in this module)
  - dependencies: [Module] (modules this module depends on)
  - progressionLevel: enum (Beginner | Intermediate | Advanced)

## Entity: Ros2Interface
- **Purpose**: Specification of ROS 2 communication elements
- **Fields**:
  - interfaceType: enum (Node | Topic | Service | Action)
  - name: string (name of interface element)
  - messageType: string (message/service/action type)
  - direction: enum (Publisher | Subscriber | Server | Client)
  - frequency: float (publishing frequency in Hz, for topics)
  - description: string (purpose and usage)

## Entity: CapstoneCapability
- **Purpose**: Core functionality of the Autonomous Humanoid project
- **Fields**:
  - capabilityId: string (unique identifier)
  - name: enum (Navigation | Perception | Voice | Planning | Manipulation | VSLAM | Control)
  - description: string (capability description)
  - requiredComponents: [string] (components needed to enable this capability)
  - successCriteria: string (how to measure success)

## Entity: DigitalTwinSystem
- **Purpose**: Digital representation of the physical robot system
- **Fields**:
  - twinId: string (unique identifier)
  - name: string (twin name)
  - simulationPlatform: enum (Gazebo | IsaacSim | Unity)
  - physicsModel: string (physics engine and parameters)
  - sensorModels: [SensorModel] (simulated sensors)
  - robotModel: RobotModel (kinematic and dynamic model)
  - visualization: enum (GazeboGUI | IsaacView | Unity3D)

## Entity: SensorModel
- **Purpose**: Simulation model for robot sensors
- **Fields**:
  - sensorId: string (unique identifier)
  - sensorType: enum (Camera | LiDAR | IMU | JointEncoder | ForceTorque)
  - topicName: string (ROS 2 topic for sensor data)
  - updateRate: float (data publishing rate in Hz)
  - noiseParameters: NoiseModel (noise characteristics)
  - physicalLocation: [float] (3D position on robot)

## Entity: RobotModel
- **Purpose**: Kinematic and dynamic model of the humanoid robot
- **Fields**:
  - robotId: string (unique identifier)
  - robotType: enum (Proxy | MiniHumanoid | PremiumHumanoid)
  - urdfPath: string (path to URDF file)
  - sdfPath: string (path to SDF file for simulation)
  - jointCount: int (number of controllable joints)
  - degreesOfFreedom: int (total DOF)
  - mass: float (total robot mass in kg)
  - dimensions: [float] (bounding box dimensions [x, y, z] in meters)

## Entity: VlaPipeline
- **Purpose**: Vision-Language-Action processing pipeline
- **Fields**:
  - pipelineId: string (unique identifier)
  - name: string (pipeline name)
  - asrComponent: AsrComponent (speech recognition component)
  - llmComponent: LlmComponent (language model component)
  - actionMapping: [ActionMapping] (mappings from intent to ROS 2 actions)
  - latencyRequirements: float (maximum allowed end-to-end latency in ms)
  - accuracyRequirements: float (minimum required accuracy percentage)

## Entity: AsrComponent
- **Purpose**: Automatic Speech Recognition component
- **Fields**:
  - componentId: string (unique identifier)
  - modelType: enum (Whisper | Custom)
  - modelSize: enum (Tiny | Base | Small | Medium | Large)
  - inputTopic: string (ROS 2 topic for audio input)
  - outputTopic: string (ROS 2 topic for text output)
  - language: string (recognized language code)
  - accuracy: float (expected recognition accuracy percentage)

## Entity: LlmComponent
- **Purpose**: Large Language Model component for intent understanding
- **Fields**:
  - componentId: string (unique identifier)
  - modelType: enum (Llama2 | Llama3 | Custom)
  - modelSize: enum (7B | 13B | 70B)
  - contextWindow: int (maximum context window in tokens)
  - responseTime: float (expected response time in ms)
  - capabilities: [string] (supported capabilities like planning, reasoning)

## Entity: ActionMapping
- **Purpose**: Mapping from natural language intent to ROS 2 actions
- **Fields**:
  - mappingId: string (unique identifier)
  - intentPattern: string (natural language pattern)
  - rosAction: Ros2Interface (corresponding ROS 2 action)
  - parameters: [ParameterMapping] (parameter mappings)
  - confidenceThreshold: float (minimum confidence for execution)

## Entity: NoiseModel
- **Purpose**: Noise characteristics for sensor simulation
- **Fields**:
  - noiseId: string (unique identifier)
  - modelType: enum (Gaussian | Uniform | Custom)
  - mean: float (mean value for Gaussian noise)
  - variance: float (variance for Gaussian noise)
  - parameters: [float] (additional model-specific parameters)

## Entity: ParameterMapping
- **Purpose**: Mapping of parameters between natural language and ROS
- **Fields**:
  - parameterId: string (unique identifier)
  - naturalLanguageTerm: string (term in natural language)
  - rosParameter: string (corresponding ROS parameter name)
  - dataType: string (data type of the parameter)
  - defaultValue: string (default value)

## Entity: SimulationBoundary
- **Purpose**: Defines the boundary between simulation and real-world components
- **Fields**:
  - boundaryId: string (unique identifier)
  - name: string (boundary name)
  - simulationComponents: [string] (components that run in simulation)
  - realComponents: [string] (components that require real hardware)
  - interfacePoints: [string] (points where simulation meets reality)
  - validationCriteria: string (criteria for validating the boundary)