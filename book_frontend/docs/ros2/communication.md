---
sidebar_position: 2
---

# ROS 2 Communication Primitives

## Understanding ROS 2 Nodes

In ROS 2, a node is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node runs independently and communicates with other nodes through the ROS 2 communication primitives.

### Key Characteristics of Nodes:
- **Process Isolation**: Each node runs as a separate process
- **Communication Interface**: Nodes provide services and topics for interaction
- **Lifecycle Management**: Nodes can be started, stopped, and monitored
- **Resource Management**: Each node manages its own resources and dependencies

### Creating a Node in Python with rclpy:
```python
import rclpy
from rclpy.node import Node

class MyRobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Node initialization code here
        self.get_logger().info('Robot controller node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: Publish/Subscribe Communication

Topics in ROS 2 implement a one-to-many communication pattern where one or more publishers send messages to one or more subscribers. This is the primary method for streaming data between nodes.

### Key Features of Topics:
- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **Data Streaming**: Ideal for continuous data like sensor readings
- **Message Types**: Defined using ROS Interface Definition Language (ROS IDL)
- **Quality of Service (QoS)**: Configurable policies for reliability, durability, etc.

### Example: Publisher and Subscriber
```python
# Publisher example
import rclpy
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.get_clock().now().nanoseconds
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

# Subscriber example
class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
```

## Services: Request/Response Communication

Services provide a synchronous, one-to-one communication pattern where a client sends a request to a server and waits for a response. This is ideal for operations that require immediate responses.

### Key Features of Services:
- **Synchronous**: Client waits for server response
- **Request/Response**: Defined message types for both request and response
- **Reliability**: Service calls are guaranteed to reach the server
- **Blocking**: Client is blocked until response is received

### Example: Service Server and Client
```python
# Service definition (example.srv)
# Request part
string name
int32 age
---
# Response part
bool success
string message

# Service server
import example_interfaces.srv
from rclpy.qos import qos_profile_default

class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        self.srv = self.create_service(
            example_interfaces.srv.SetBool,
            'set_boolean',
            self.set_boolean_callback)

    def set_boolean_callback(self, request, response):
        response.success = True
        response.message = f'Received request: {request.data}'
        self.get_logger().info(f'Returning: {response.success} % {response.message}')
        return response

# Service client
class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        self.cli = self.create_client(example_interfaces.srv.SetBool, 'set_boolean')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = example_interfaces.srv.SetBool.Request()

    def send_request(self, data):
        self.req.data = data
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
```

## Actions: Goal-Oriented Communication

Actions provide a communication pattern for long-running tasks that require feedback and status updates. They combine features of both topics and services.

### Key Features of Actions:
- **Goal Management**: Send goals, receive feedback, and get results
- **Cancelation**: Ability to cancel long-running operations
- **Feedback**: Continuous updates during execution
- **Status Tracking**: Monitor the state of long-running tasks

## Message Passing and Real-time Constraints

ROS 2 provides mechanisms to handle real-time constraints through Quality of Service (QoS) policies. These policies allow fine-tuning of communication behavior based on the requirements of the application.

### QoS Policies:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local
- **Deadline**: Time constraints for message delivery
- **Liveliness**: Detection of active publishers/subscribers
- **History**: Keep all vs. keep last N messages

## Using rclpy to Connect Python AI Agents to Robot Controllers

rclpy is the Python client library for ROS 2 that allows Python-based AI agents to interact with the ROS 2 system. It provides the necessary APIs to create nodes, publishers, subscribers, services, and actions.

### Key Benefits of rclpy for AI Agents:
- **Python Integration**: Leverage Python's rich AI/ML ecosystem
- **ROS 2 Compatibility**: Full access to ROS 2 communication primitives
- **Simplicity**: Easy-to-use Python APIs for ROS 2 concepts
- **Flexibility**: Support for all ROS 2 communication patterns

### Example: AI Agent Connecting to Robot Controller
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

class AIAgent(Node):
    def __init__(self):
        super().__init__('ai_agent')

        # Subscribe to sensor data
        self.sensor_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish commands to robot
        self.command_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # AI processing timer
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)

        self.current_joint_states = None

    def joint_state_callback(self, msg):
        self.current_joint_states = msg
        # Process sensor data with AI algorithms
        self.process_sensor_data(msg)

    def ai_processing_callback(self):
        if self.current_joint_states is not None:
            # Apply AI decision-making
            trajectory_command = self.generate_trajectory_command()
            self.command_publisher.publish(trajectory_command)

    def process_sensor_data(self, joint_state):
        # Apply AI algorithms to process sensor data
        # This could include perception, state estimation, etc.
        pass

    def generate_trajectory_command(self):
        # Generate commands based on AI decision-making
        # This could include path planning, control algorithms, etc.
        trajectory = JointTrajectory()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3']

        point = JointTrajectoryPoint()
        point.positions = [1.0, 2.0, 3.0]  # Example positions
        point.time_from_start.sec = 1
        trajectory.points.append(point)

        return trajectory

def main(args=None):
    rclpy.init(args=args)
    ai_agent = AIAgent()

    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 communication primitives (Nodes, Topics, Services, and Actions) provide the essential infrastructure for connecting AI agents to robot controllers. Using rclpy, Python-based AI systems can seamlessly integrate with ROS 2's distributed architecture, enabling sophisticated robot control applications. Understanding these communication patterns is crucial for developing effective AI-robot interfaces.

## Next Steps

Continue to the next chapter to learn about [Humanoid Representation with URDF](../ros2/urdf), where you'll explore how to model humanoid robots with links, joints, and sensors for both simulation and real-world applications.