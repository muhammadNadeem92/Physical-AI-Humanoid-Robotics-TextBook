# Data Model: Capstone - The Autonomous Humanoid System

**Feature**: Module 8: Capstone - The Autonomous Humanoid System
**Date**: 2025-12-18
**Input**: Feature specification from `/specs/008-capstone-autonomous-humanoid/spec.md`

## Entity Definitions

### 1. VoiceCommand
**Description**: Natural language input that represents user intent, containing task descriptions and object references
**Attributes**:
- id: Unique identifier for the command
- content: Text content of the voice command
- timestamp: When the command was received
- confidence: Confidence level of speech-to-text conversion
- source: Origin of the command (e.g., microphone, simulation input)

**Validation Rules**:
- content must not be empty
- confidence must be between 0.0 and 1.0
- timestamp must be in ISO 8601 format

### 2. TaskPlan
**Description**: Structured sequence of actions derived from voice commands, containing navigation, manipulation, and safety constraints
**Attributes**:
- id: Unique identifier for the task plan
- actions: Array of action objects in execution order
- constraints: Safety and execution constraints
- validation_status: Status of plan validation
- created_at: Timestamp when plan was created

**Relationships**:
- Contains multiple Action objects
- References VoiceCommand that originated it

**Validation Rules**:
- actions array must not be empty
- all actions must have valid types and parameters
- constraints must be consistent with safety requirements

### 3. PerceptionOutput
**Description**: Detected objects and environmental state derived from vision sensors, including object types, locations, and properties
**Attributes**:
- id: Unique identifier for the perception result
- objects: Array of detected objects with properties
- environment_state: General environmental conditions
- timestamp: When the perception was captured
- confidence_threshold: Minimum confidence for valid detections

**Relationships**:
- Generated from sensor data inputs
- Used by WorldState to update environment representation

**Validation Rules**:
- objects array may be empty but must exist
- each object must have a unique identifier within the result
- timestamp must be in ISO 8601 format

### 4. WorldState
**Description**: Integrated representation of the environment that combines perception data with task context and safety boundaries
**Attributes**:
- id: Unique identifier for the world state
- objects: Current state of known objects in environment
- spatial_relations: Relationships between objects
- safety_boundaries: Current safety zones and restrictions
- last_updated: Timestamp of last state update

**Relationships**:
- Incorporates multiple PerceptionOutput objects
- Associated with active TaskPlan
- Updated by perception and action feedback

**Validation Rules**:
- Must be updated within specified time intervals
- Safety boundaries must be maintained at all times
- Object positions must be geometrically consistent

### 5. ActionGraph
**Description**: Sequence of ROS 2 actions that implement the task plan, with dependencies and monitoring requirements
**Attributes**:
- id: Unique identifier for the action graph
- actions: Array of action nodes
- dependencies: Relationships between actions
- monitoring_requirements: What to monitor during execution
- status: Current execution status of the graph

**Relationships**:
- Derived from TaskPlan
- Contains multiple ROS 2 Action objects
- Monitored by ExecutionState

**Validation Rules**:
- All dependencies must reference valid action nodes
- No circular dependencies allowed
- All actions must be executable in the given order

### 6. ExecutionState
**Description**: Current status of task execution, including progress, errors, and recovery requirements
**Attributes**:
- id: Unique identifier for the execution state
- current_action: Currently executing action
- progress: Percentage of completion (0-100)
- errors: Array of errors encountered
- recovery_status: Current recovery state if applicable

**Relationships**:
- Associated with ActionGraph being executed
- Updates WorldState during execution
- Generates feedback for monitoring

**Validation Rules**:
- Progress must be between 0 and 100
- Current action must be part of the associated ActionGraph
- Errors must be properly classified and timestamped

## State Transitions

### TaskPlan States
- **CREATING**: Plan is being generated from voice command
- **VALIDATING**: Plan is being checked for safety and feasibility
- **VALIDATED**: Plan passed all validation checks
- **EXECUTING**: Plan is being executed by the robot
- **COMPLETED**: Plan execution finished successfully
- **FAILED**: Plan execution failed and cannot continue
- **CANCELLED**: Plan execution was cancelled by user or safety system

### ExecutionState States
- **IDLE**: No task execution in progress
- **INITIALIZING**: Preparing to execute a task plan
- **RUNNING**: Task plan execution is active
- **PAUSED**: Execution paused (e.g., for safety check)
- **ERROR**: An error occurred during execution
- **RECOVERING**: Attempting to recover from error
- **COMPLETED**: Task execution completed successfully
- **STOPPED**: Execution stopped by user or safety system

## Relationships

```
VoiceCommand --[generates]--> TaskPlan
TaskPlan --[contains]--> ActionGraph
ActionGraph --[executes via]--> ExecutionState
PerceptionOutput --[updates]--> WorldState
WorldState --[influences]--> TaskPlan (for adaptation)
ExecutionState --[updates]--> WorldState
```

## Data Flow Patterns

### Voice Processing Flow
VoiceCommand → Speech-to-Text → Intent Parsing → TaskPlan Generation

### Perception Flow
Sensor Data → PerceptionOutput → WorldState Update → Task Adaptation

### Execution Flow
TaskPlan → ActionGraph → ExecutionState → Action Execution → WorldState Updates

## Constraints

1. **Safety Constraints**: All actions must maintain safety boundaries as defined in the specification
2. **Temporal Constraints**: WorldState must be updated within specified time intervals to remain valid
3. **Consistency Constraints**: Object positions in WorldState must be geometrically consistent
4. **Validation Constraints**: TaskPlans must be validated before execution can begin
5. **Monitoring Constraints**: ExecutionState must continuously monitor for safety violations