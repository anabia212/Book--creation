# API Contract: Vision-Language-Action (VLA) Systems

## Overview
API contract for the Vision-Language-Action (VLA) system that enables humanoid robots to understand voice commands, plan actions using LLMs, and execute tasks through ROS 2 integration.

## API Endpoints

### Voice Command Processing
- **Endpoint**: `POST /api/vla/voice-command`
- **Purpose**: Submit a voice command for processing
- **Request**:
  - `audioData`: Base64 encoded audio data or URL to audio file
  - `userId`: Identifier for the user issuing the command
  - `robotId`: Identifier for the target robot
- **Response**:
  - `commandId`: Unique identifier for the processed command
  - `transcribedText`: Text representation of the spoken command
  - `confidenceScore`: Confidence level of speech recognition (0.0-1.0)
  - `intentClassification`: Structured intent derived from the command
  - `status`: Processing status (received, processing, processed, failed)

### Cognitive Planning
- **Endpoint**: `POST /api/vla/cognitive-plan`
- **Purpose**: Generate a cognitive plan from a natural language goal
- **Request**:
  - `goalDescription`: Natural language goal description
  - `context`: Environmental and state context for planning
  - `robotCapabilities`: Available robot capabilities
- **Response**:
  - `planId`: Unique identifier for the generated plan
  - `actionSequence`: Ordered list of actions to execute
  - `reasoningSteps`: Explanation of the planning process
  - `estimatedDuration`: Expected time to complete the plan
  - `validationStatus`: Whether the plan is valid for execution

### Action Execution
- **Endpoint**: `POST /api/vla/execute-action`
- **Purpose**: Execute an action sequence on a robot
- **Request**:
  - `actionSequenceId`: Identifier for the sequence to execute
  - `robotId`: Identifier for the target robot
  - `executionContext`: Current state and environmental conditions
- **Response**:
  - `executionId`: Unique identifier for this execution
  - `status`: Execution status (pending, executing, completed, failed)
  - `progress`: Progress percentage (0-100)
  - `feedbackMessage`: Human-readable feedback about execution

### Execution Status
- **Endpoint**: `GET /api/vla/execution/{executionId}`
- **Purpose**: Get the current status of an ongoing execution
- **Response**:
  - `executionId`: Identifier for the execution
  - `status`: Current execution status
  - `progress`: Progress percentage (0-100)
  - `completedActions`: List of successfully completed actions
  - `failedActions`: List of failed actions with error details
  - `estimatedTimeRemaining`: Estimated time until completion

## Data Models

### VoiceCommand
```json
{
  "commandId": "string",
  "audioData": "base64-encoded string or URL",
  "transcribedText": "string",
  "confidenceScore": "number (0.0-1.0)",
  "intentClassification": "string",
  "timestamp": "ISO 8601 datetime",
  "processingStatus": "enum: received, processing, processed, failed"
}
```

### CognitivePlan
```json
{
  "planId": "string",
  "goalDescription": "string",
  "actionSequence": [
    {
      "actionId": "string",
      "actionType": "string",
      "parameters": "object",
      "dependencies": ["string"]
    }
  ],
  "reasoningSteps": ["string"],
  "executionContext": {
    "currentState": "object",
    "environmentalConditions": "object",
    "robotCapabilities": ["string"],
    "constraints": ["string"]
  },
  "estimatedDuration": "number (seconds)",
  "validationStatus": "enum: valid, invalid, requires_revision"
}
```

### ExecutionResult
```json
{
  "executionId": "string",
  "commandId": "string",
  "planId": "string",
  "status": "enum: pending, executing, completed, failed, interrupted",
  "progress": "number (0-100)",
  "completedActions": ["string"],
  "failedActions": [
    {
      "actionId": "string",
      "error": "string",
      "timestamp": "ISO 8601 datetime"
    }
  ],
  "feedbackMessage": "string",
  "completionTime": "ISO 8601 datetime",
  "estimatedTimeRemaining": "number (seconds)"
}
```

## Error Handling

### Standard Error Response
```json
{
  "errorId": "string",
  "message": "string",
  "code": "number",
  "timestamp": "ISO 8601 datetime",
  "details": "object (optional)"
}
```

### Common Error Codes
- `400`: Bad Request - Invalid request parameters
- `401`: Unauthorized - Authentication required
- `403`: Forbidden - Insufficient permissions
- `404`: Not Found - Requested resource not found
- `422`: Unprocessable Entity - Request parameters valid but operation cannot be completed
- `500`: Internal Server Error - Server-side error
- `503`: Service Unavailable - External service (e.g., LLM, ROS) unavailable

## Authentication
- All endpoints require authentication using Bearer tokens
- Tokens should be included in the `Authorization` header

## Rate Limiting
- API requests are limited to 100 requests per minute per user
- Exceeding the limit results in a 429 status code

## Versioning
- API version 1.0 is specified using the header `API-Version: 1.0`
- Breaking changes will result in a new major version