# Data Model: Vision-Language-Action (VLA) Systems

## Core Entities

### VoiceCommand
- **Description**: Represents a spoken command from a user that needs processing
- **Attributes**:
  - `id`: Unique identifier for the command
  - `audioData`: Audio input data or reference to audio file
  - `transcribedText`: Text representation of the spoken command
  - `confidenceScore`: Confidence level of speech recognition (0.0-1.0)
  - `intentClassification`: Structured intent derived from the command
  - `timestamp`: When the command was received
  - `processingStatus`: Current status (received, processing, processed, failed)

### CognitivePlan
- **Description**: Represents the high-level reasoning output that breaks down goals into executable steps
- **Attributes**:
  - `id`: Unique identifier for the plan
  - `goalDescription`: Original natural language goal
  - `actionSequence`: Ordered list of actions to execute
  - `reasoningSteps`: Explanation of the planning process
  - `executionContext`: Environmental and state constraints
  - `createdAt`: When the plan was generated
  - `validityPeriod`: How long the plan remains valid

### ActionSequence
- **Description**: Represents a series of executable robot actions derived from natural language goals
- **Attributes**:
  - `id`: Unique identifier for the sequence
  - `actions`: Array of individual action objects
  - `executionOrder`: Order in which actions should be executed
  - `dependencies`: Relationships between actions
  - `successCriteria`: Conditions that define successful execution
  - `estimatedDuration`: Expected time to complete all actions

### ExecutionContext
- **Description**: Represents the current state and environmental conditions that affect action planning
- **Attributes**:
  - `id`: Unique identifier for the context
  - `currentState`: Current robot state (position, battery, etc.)
  - `environmentalConditions`: Environmental constraints (obstacles, lighting, etc.)
  - `robotCapabilities`: Available robot capabilities for action execution
  - `constraints`: Safety and operational constraints
  - `updatedAt`: When the context was last updated

### VLAExecutionResult
- **Description**: Represents the outcome of executing a VLA command sequence
- **Attributes**:
  - `id`: Unique identifier for the execution result
  - `commandId`: Reference to the original voice command
  - `planId`: Reference to the cognitive plan executed
  - `executionStatus`: Overall status (success, partial, failed)
  - `completedActions`: List of successfully completed actions
  - `failedActions`: List of failed actions with error details
  - `feedbackMessage`: Human-readable feedback about execution
  - `completionTime`: When execution was completed

## Relationships

### VoiceCommand → CognitivePlan
- **Relationship**: One-to-Many (one command can generate multiple plan attempts)
- **Description**: A voice command is processed to generate cognitive plans

### CognitivePlan → ActionSequence
- **Relationship**: One-to-One
- **Description**: A cognitive plan contains one action sequence

### ActionSequence → ExecutionContext
- **Relationship**: Many-to-One
- **Description**: Multiple action sequences may share similar execution contexts

### ActionSequence → VLAExecutionResult
- **Relationship**: One-to-Many
- **Description**: An action sequence can be executed multiple times with different results

## State Transitions

### VoiceCommand States
- `received` → `processing` → `processed` | `failed`
- Validation occurs during processing state

### CognitivePlan States
- `generating` → `validating` → `approved` | `rejected`
- Plans may be revised if rejected

### ActionSequence States
- `planned` → `executing` → `completed` | `interrupted` | `failed`
- Execution can be paused and resumed

## Validation Rules

### VoiceCommand Validation
- `transcribedText` must not be empty
- `confidenceScore` must be between 0.0 and 1.0
- `intentClassification` must match defined intent types

### CognitivePlan Validation
- `actionSequence` must contain at least one valid action
- `reasoningSteps` must provide sufficient explanation
- Plan must be consistent with execution context

### ActionSequence Validation
- All actions in sequence must be valid ROS 2 actions
- Dependencies must form a valid execution graph
- Estimated duration must be reasonable

## Indexes and Performance Considerations

### Primary Indexes
- VoiceCommand: `id`, `timestamp`
- CognitivePlan: `id`, `createdAt`
- ActionSequence: `id`
- ExecutionContext: `id`, `updatedAt`

### Query Patterns
- Recent voice commands by timestamp
- Plans by goal description
- Execution results by status
- Context updates by robot or environment