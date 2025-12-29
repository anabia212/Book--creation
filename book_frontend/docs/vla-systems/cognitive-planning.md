---
title: Cognitive Planning with LLMs
sidebar_label: Cognitive Planning with LLMs
---

# Cognitive Planning with LLMs

## Introduction to Cognitive Planning in Robotics

Cognitive planning in robotics refers to the high-level reasoning process that translates natural language goals into executable action sequences. This involves complex decision-making, task decomposition, and strategic planning that enables humanoid robots to understand and execute complex commands that require multiple steps and environmental awareness.

### Key Aspects of Cognitive Planning

- **Task Decomposition**: Breaking complex goals into manageable subtasks
- **Strategic Reasoning**: Planning optimal sequences of actions
- **Environmental Awareness**: Incorporating real-time environmental information
- **Constraint Handling**: Managing safety and operational constraints
- **Adaptive Planning**: Adjusting plans based on execution feedback

## Translating Natural Language Goals into ROS 2 Action Sequences

The translation of natural language goals into ROS 2 action sequences involves several sophisticated processes that bridge human communication with robotic execution.

### Natural Language Understanding Pipeline

The process begins with interpreting natural language goals and converting them into structured representations that robots can understand:

```python
import openai
from typing import Dict, List, Optional
from dataclasses import dataclass

@dataclass
class RobotAction:
    action_type: str
    parameters: Dict[str, any]
    priority: int = 0
    constraints: List[str] = None

@dataclass
class CognitivePlan:
    goal_description: str
    action_sequence: List[RobotAction]
    reasoning_steps: List[str]
    estimated_duration: float
    confidence_score: float

class NaturalLanguagePlanner:
    def __init__(self, llm_client):
        self.llm_client = llm_client
        self.action_registry = self._initialize_action_registry()

    def _initialize_action_registry(self) -> Dict[str, str]:
        """Initialize the registry of available robot actions"""
        return {
            "navigate_to": "Navigation action for moving to locations",
            "pick_object": "Manipulation action for grasping objects",
            "place_object": "Manipulation action for placing objects",
            "detect_object": "Perception action for identifying objects",
            "open_gripper": "Manipulation action for opening gripper",
            "close_gripper": "Manipulation action for closing gripper",
            "rotate_wrist": "Manipulation action for wrist rotation",
            "move_arm": "Manipulation action for arm positioning"
        }

    def plan_from_natural_language(self, goal: str) -> Optional[CognitivePlan]:
        """Generate a cognitive plan from a natural language goal"""
        # Use LLM to decompose the goal and identify necessary actions
        prompt = f"""
        Goal: {goal}

        Please decompose this goal into a sequence of robot actions from the following list:
        {list(self.action_registry.keys())}

        For each action, specify:
        1. Action type
        2. Parameters (object names, locations, etc.)
        3. Priority/sequence
        4. Any constraints

        Provide the reasoning steps you used to decompose the goal.
        Estimate the total duration in seconds.
        Provide a confidence score between 0 and 1.

        Return the result in JSON format.
        """

        try:
            response = self.llm_client.chat.completions.create(
                model="gpt-4",
                messages=[{"role": "user", "content": prompt}],
                response_format={"type": "json_object"}
            )

            plan_data = response.choices[0].message.content
            import json
            plan_dict = json.loads(plan_data)

            # Convert to CognitivePlan object
            action_sequence = [
                RobotAction(
                    action_type=action['action_type'],
                    parameters=action.get('parameters', {}),
                    priority=action.get('priority', 0),
                    constraints=action.get('constraints', [])
                )
                for action in plan_dict['action_sequence']
            ]

            return CognitivePlan(
                goal_description=goal,
                action_sequence=action_sequence,
                reasoning_steps=plan_dict['reasoning_steps'],
                estimated_duration=plan_dict['estimated_duration'],
                confidence_score=plan_dict['confidence_score']
            )
        except Exception as e:
            print(f"Error generating plan: {e}")
            return None
```

### ROS 2 Action Integration

The generated plans need to be integrated with ROS 2 action servers for execution:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from your_msgs.action import NavigateToPose
from your_msgs.action import ManipulateObject
from your_msgs.action import DetectObject

class PlanExecutorNode(Node):
    def __init__(self):
        super().__init__('plan_executor_node')

        # Action clients for different robot capabilities
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(self, ManipulateObject, 'manipulate_object')
        self.detect_client = ActionClient(self, DetectObject, 'detect_object')

        # Publisher for plan status
        self.status_pub = self.create_publisher(String, 'plan_status', 10)

        # Timer for plan execution
        self.plan_timer = self.create_timer(0.1, self.plan_execution_callback)

        self.current_plan = None
        self.current_step = 0

    def execute_plan(self, plan: CognitivePlan):
        """Execute a cognitive plan"""
        self.current_plan = plan
        self.current_step = 0
        self.get_logger().info(f"Starting execution of plan: {plan.goal_description}")

        if self.current_plan.action_sequence:
            self._execute_next_step()

    def _execute_next_step(self):
        """Execute the next step in the plan"""
        if self.current_step >= len(self.current_plan.action_sequence):
            self._plan_complete()
            return

        action = self.current_plan.action_sequence[self.current_step]

        self.get_logger().info(f"Executing action {self.current_step + 1}: {action.action_type}")

        # Execute the appropriate action based on type
        if action.action_type == "navigate_to":
            self._execute_navigation_action(action)
        elif action.action_type == "pick_object":
            self._execute_manipulation_action(action)
        elif action.action_type == "detect_object":
            self._execute_detection_action(action)
        else:
            self.get_logger().warn(f"Unknown action type: {action.action_type}")
            self.current_step += 1
            self._execute_next_step()

    def _execute_navigation_action(self, action):
        """Execute a navigation action"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._convert_to_pose(action.parameters)

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._navigation_result_callback)

    def _execute_manipulation_action(self, action):
        """Execute a manipulation action"""
        goal_msg = ManipulateObject.Goal()
        goal_msg.operation = action.parameters.get('operation', 'pick')
        goal_msg.object_name = action.parameters.get('object_name', '')
        goal_msg.pose = self._convert_to_pose(action.parameters)

        self.manip_client.wait_for_server()
        future = self.manip_client.send_goal_async(goal_msg)
        future.add_done_callback(self._manipulation_result_callback)

    def _navigation_result_callback(self, future):
        """Handle navigation action result"""
        result = future.result()
        if result.success:
            self.get_logger().info("Navigation successful")
            self.current_step += 1
            self._execute_next_step()
        else:
            self.get_logger().error("Navigation failed")
            self._plan_failed()

    def _manipulation_result_callback(self, future):
        """Handle manipulation action result"""
        result = future.result()
        if result.success:
            self.get_logger().info("Manipulation successful")
            self.current_step += 1
            self._execute_next_step()
        else:
            self.get_logger().error("Manipulation failed")
            self._plan_failed()

    def _plan_complete(self):
        """Handle plan completion"""
        self.get_logger().info("Plan completed successfully")
        status_msg = String()
        status_msg.data = f"Plan completed: {self.current_plan.goal_description}"
        self.status_pub.publish(status_msg)
        self.current_plan = None

    def _plan_failed(self):
        """Handle plan failure"""
        self.get_logger().error("Plan execution failed")
        status_msg = String()
        status_msg.data = f"Plan failed: {self.current_plan.goal_description}"
        self.status_pub.publish(status_msg)
        self.current_plan = None
```

## High-Level Reasoning and Task Decomposition

High-level reasoning involves breaking down complex goals into manageable subtasks that can be executed sequentially or in parallel.

### Task Decomposition Strategies

1. **Hierarchical Decomposition**: Breaking tasks into subtasks in a tree structure
2. **Temporal Decomposition**: Sequencing tasks based on time dependencies
3. **Functional Decomposition**: Grouping tasks by function or capability
4. **Spatial Decomposition**: Organizing tasks based on spatial relationships

### Example: Complex Goal Decomposition

Consider the goal "Go to the kitchen, find the red cup, and bring it to the living room." The decomposition might look like:

```python
def decompose_complex_goal(goal: str) -> CognitivePlan:
    """
    Example of decomposing a complex goal into subtasks
    Goal: "Go to the kitchen, find the red cup, and bring it to the living room."
    """
    plan = CognitivePlan(
        goal_description=goal,
        action_sequence=[
            RobotAction(
                action_type="navigate_to",
                parameters={"location": "kitchen"},
                priority=1
            ),
            RobotAction(
                action_type="detect_object",
                parameters={"object_type": "cup", "color": "red"},
                priority=2
            ),
            RobotAction(
                action_type="pick_object",
                parameters={"object_type": "cup", "color": "red"},
                priority=3
            ),
            RobotAction(
                action_type="navigate_to",
                parameters={"location": "living_room"},
                priority=4
            ),
            RobotAction(
                action_type="place_object",
                parameters={"location": "table"},
                priority=5
            )
        ],
        reasoning_steps=[
            "1. Identify the target object: red cup",
            "2. Identify the starting location: current position",
            "3. Identify the object location: kitchen",
            "4. Identify the destination: living room",
            "5. Plan navigation to kitchen",
            "6. Plan object detection and identification",
            "7. Plan object pickup",
            "8. Plan navigation to living room",
            "9. Plan object placement"
        ],
        estimated_duration=120.0,  # seconds
        confidence_score=0.85
    )
    return plan
```

### Constraint Handling

Cognitive planning must consider various constraints:

- **Physical Constraints**: Robot kinematics, payload limits
- **Environmental Constraints**: Obstacles, restricted areas
- **Safety Constraints**: Collision avoidance, speed limits
- **Temporal Constraints**: Deadlines, time windows

```python
class ConstraintValidator:
    def __init__(self):
        self.constraints = {
            "max_payload": 5.0,  # kg
            "max_navigation_speed": 1.0,  # m/s
            "reachable_distance": 1.5,  # m
            "safe_zone_radius": 0.5  # m
        }

    def validate_action(self, action: RobotAction, robot_state: Dict) -> bool:
        """Validate an action against constraints"""
        if action.action_type == "pick_object":
            object_weight = action.parameters.get("weight", 0)
            if object_weight > self.constraints["max_payload"]:
                return False, f"Object too heavy: {object_weight}kg > {self.constraints['max_payload']}kg"

        if action.action_type == "navigate_to":
            target_distance = action.parameters.get("distance", 0)
            if target_distance > self.constraints["reachable_distance"]:
                return False, f"Target too far: {target_distance}m > {self.constraints['reachable_distance']}m"

        return True, "Valid"
```

## Integration with Large Language Models

Large Language Models (LLMs) play a crucial role in cognitive planning by providing the reasoning capabilities needed to decompose complex goals.

### LLM-Powered Planning Architecture

```python
class LLMPlanner:
    def __init__(self, model_name: str = "gpt-4"):
        self.model_name = model_name
        self.action_descriptions = self._get_action_descriptions()

    def _get_action_descriptions(self) -> Dict[str, str]:
        """Get detailed descriptions of available actions"""
        return {
            "navigate_to": "Move the robot to a specified location. Parameters: target_location (string), speed (optional float)",
            "detect_object": "Use robot's perception system to detect an object. Parameters: object_type (string), color (optional string), search_area (optional string)",
            "pick_object": "Grasp an object with the robot's manipulator. Parameters: object_name (string), grasp_type (optional string)",
            "place_object": "Release an object at a specified location. Parameters: target_location (string), placement_type (optional string)",
            "open_gripper": "Open the robot's gripper. Parameters: none",
            "close_gripper": "Close the robot's gripper. Parameters: force (optional float)"
        }

    def generate_plan_with_llm(self, goal: str, context: Dict = None) -> CognitivePlan:
        """Generate a plan using LLM with context awareness"""
        context_str = self._format_context(context) if context else "No specific context provided."

        prompt = f"""
        Context: {context_str}

        Goal: {goal}

        Available actions and their descriptions:
        {self.action_descriptions}

        Please create a detailed cognitive plan that:
        1. Breaks down the goal into a sequence of specific actions
        2. Specifies the parameters for each action
        3. Considers the provided context
        4. Includes reasoning for why each action is necessary
        5. Estimates the duration for the entire plan
        6. Provides a confidence score based on clarity of the goal

        Return the plan in JSON format with the following structure:
        {{
          "action_sequence": [
            {{
              "action_type": "string",
              "parameters": {{"param_name": "value"}},
              "reasoning": "string"
            }}
          ],
          "reasoning_steps": ["string"],
          "estimated_duration": float,
          "confidence_score": float
        }}
        """

        # This would call the actual LLM API
        # response = openai.ChatCompletion.create(model=self.model_name, prompt=prompt)

        # For this example, we'll return a mock response
        return self._create_mock_plan(goal)

    def _create_mock_plan(self, goal: str) -> CognitivePlan:
        """Create a mock plan for demonstration purposes"""
        return CognitivePlan(
            goal_description=goal,
            action_sequence=[
                RobotAction(action_type="navigate_to", parameters={"location": "kitchen"}),
                RobotAction(action_type="detect_object", parameters={"object_type": "cup", "color": "red"}),
                RobotAction(action_type="pick_object", parameters={"object_name": "red_cup"}),
                RobotAction(action_type="navigate_to", parameters={"location": "living_room"}),
                RobotAction(action_type="place_object", parameters={"location": "table"})
            ],
            reasoning_steps=[
                "1. Navigate to the kitchen where the cup is likely to be found",
                "2. Detect the red cup using the perception system",
                "3. Pick up the detected red cup",
                "4. Navigate to the living room as specified in the goal",
                "5. Place the cup on the table"
            ],
            estimated_duration=180.0,
            confidence_score=0.9
        )
```

## Practical Implementation Considerations

### Error Handling and Plan Recovery

```python
class RobustPlanExecutor:
    def __init__(self):
        self.max_retries = 3
        self.recovery_strategies = {
            "navigation_failure": self._handle_navigation_failure,
            "detection_failure": self._handle_detection_failure,
            "manipulation_failure": self._handle_manipulation_failure
        }

    def _handle_navigation_failure(self, current_plan: CognitivePlan, failed_step: int):
        """Handle navigation failure with alternative strategies"""
        # Try alternative route
        # Ask for human assistance
        # Modify plan to skip this step if possible
        pass

    def _handle_detection_failure(self, current_plan: CognitivePlan, failed_step: int):
        """Handle object detection failure"""
        # Increase search area
        # Change detection parameters
        # Ask for more specific information about object location
        pass

    def _handle_manipulation_failure(self, current_plan: CognitivePlan, failed_step: int):
        """Handle manipulation failure"""
        # Adjust grasp parameters
        # Re-approach the object
        # Consider alternative manipulation strategies
        pass
```

## Best Practices for Cognitive Planning

### Plan Validation

- **Pre-execution Validation**: Verify plan feasibility before execution
- **Runtime Monitoring**: Continuously monitor plan execution
- **Dynamic Adjustment**: Modify plans based on changing conditions
- **Fallback Mechanisms**: Implement graceful degradation when plans fail

### Performance Optimization

- **Plan Caching**: Cache frequently used plans for efficiency
- **Parallel Execution**: Execute independent subtasks in parallel when possible
- **Plan Simplification**: Simplify plans when full complexity isn't needed
- **Learning from Execution**: Update planning based on execution outcomes

## Summary

Cognitive planning with LLMs enables humanoid robots to understand complex natural language goals and translate them into executable action sequences. By leveraging the reasoning capabilities of large language models, robots can perform sophisticated task decomposition and strategic planning that was previously only possible with extensive manual programming.

## Exercises and Self-Assessment

### Exercise 1: LLM-Based Planning Implementation
1. Set up a large language model API for planning (e.g., OpenAI GPT-4)
2. Create a prompt template for task decomposition
3. Implement the planning pipeline from natural language to action sequences
4. Test with various complex goals and evaluate plan quality

### Exercise 2: Constraint Handling
1. Implement a constraint validation system for robot actions
2. Test with physical, environmental, and safety constraints
3. Create recovery strategies for constraint violations
4. Evaluate the robustness of the planning system

### Self-Assessment Questions
1. How do LLMs contribute to cognitive planning in robotics?
2. What are the key components of task decomposition?
3. Explain the process of translating natural language goals to ROS 2 actions.
4. What constraints need to be considered in cognitive planning?
5. How can plans be validated before execution?

## Next Steps

- Review [Voice-to-Action Interfaces](./voice-to-action.md) for command processing techniques
- Continue to [Capstone: The Autonomous Humanoid](./autonomous-humanoid.md) for complete system integration