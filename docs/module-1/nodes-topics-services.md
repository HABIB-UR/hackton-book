---
sidebar_position: 2
---

# Nodes, Topics, Services

## Understanding ROS 2 Communication

ROS 2 provides several communication patterns that enable nodes to interact with each other. These patterns are designed for different types of interactions, from continuous data streams to request-response interactions. Understanding these patterns is crucial for designing effective robotic systems.

## Nodes: The Building Blocks

In ROS 2, a **node** is a process that performs computation. Nodes are the fundamental building blocks of a ROS 2 system. Each node typically handles a specific task or function within the robot system. Multiple nodes run simultaneously in a ROS 2 system, and they communicate with each other through various communication mechanisms.

### Key Characteristics of Nodes:
- Each node runs as a separate process
- Nodes can be written in different programming languages (C++, Python, etc.)
- Nodes are organized into packages for easier management
- Nodes can run on the same machine or be distributed across multiple machines

## Topics: Publish/Subscribe Communication

Topics enable one-way, asynchronous communication between nodes using a publish/subscribe model. This pattern is ideal for continuous data streams such as sensor data, robot state information, or commands.

### How Topics Work:
- **Publisher**: A node that sends data to a topic
- **Subscriber**: A node that receives data from a topic
- **Topic**: A named channel to which messages are sent

### Characteristics of Topics:
- **Asynchronous**: Publishers and subscribers don't need to be synchronized
- **Many-to-many**: Multiple publishers can send to a topic, and multiple subscribers can receive from a topic
- **Unidirectional**: Data flows in one direction (publisher → topic → subscriber)
- **Best-effort**: Messages may be lost if the system is overloaded
- **Real-time friendly**: Low latency communication

### Example: Sensor Data
```python
# Publisher node - IMU sensor
import rclpy
from sensor_msgs.msg import Imu

def publisher_callback():
    msg = Imu()
    # Populate message with sensor data
    publisher.publish(msg)

# Subscriber node - State estimator
def subscription_callback(msg):
    # Process incoming IMU data
    process_imu_data(msg)
```

## Services: Request/Response Communication

Services provide synchronous, bidirectional communication between nodes using a request/response model. This pattern is ideal for actions that require a specific response, such as configuration changes or computational tasks.

### How Services Work:
- **Service Client**: A node that sends a request and waits for a response
- **Service Server**: A node that receives requests and sends responses
- **Service**: A named interface with a defined request/response structure

### Characteristics of Services:
- **Synchronous**: The client waits for the server to respond
- **One-to-one**: Each request is handled by one server
- **Reliable**: Requests and responses are guaranteed to be delivered
- **Blocking**: The client blocks until it receives a response

### Example: Robot Control
```python
# Service Server - Movement controller
from example_interfaces.srv import SetBool

def handle_move_request(request, response):
    if request.data:  # If request is to move
        success = move_robot()
        response.success = success
        response.message = "Move command executed"
    return response

# Service Client - High-level controller
def send_move_command():
    request = SetBool.Request()
    request.data = True
    future = client.call_async(request)
    # Wait for response
```

## Actions: Goal-Based Communication

Actions are used for long-running tasks that may take significant time to complete. They combine features of both topics and services, providing feedback during execution and the ability to cancel tasks.

### How Actions Work:
- **Action Client**: Requests a goal and can monitor progress
- **Action Server**: Executes goals and provides feedback
- **Goal**: A specific task to be performed
- **Feedback**: Updates on the progress of the goal
- **Result**: The final outcome of the goal

### Characteristics of Actions:
- **Long-running**: Designed for tasks that take time
- **Cancellable**: Clients can cancel goals in progress
- **Feedback**: Provides updates during execution
- **Status reporting**: Clients can monitor goal status

## Humanoid Motion Signal Flow

In humanoid robots, communication patterns follow specific patterns to ensure coordinated movement and control. Let's examine how these patterns work together:

![Humanoid Robot Communication Flow](/images/communication/humanoid-communication.svg)

### Perception-Action Loop
1. **Sensors publish** data (topics): Joint encoders, IMU, cameras, force sensors
2. **Perception nodes subscribe** to sensor data and publish processed information
3. **Planning nodes** receive processed data and publish commands
4. **Control nodes** subscribe to commands and send low-level control signals to actuators
5. **Actuators** execute commands and may publish status updates

### Example Architecture
```
[Joint Encoders] → /joint_states (topic) → [State Estimator]
[IMU Sensor] → /imu/data (topic) → [Balance Controller]
[Camera] → /image_raw (topic) → [Vision Processor] → /detected_objects (topic)
[High-level Planner] → /walk_goal (action) → [Walking Controller]
[UI Node] → /set_pose (service) → [Pose Controller]
```

## Communication Design Patterns

### 1. Sensor Fusion
Multiple sensors publish to a common topic or to separate topics that are subscribed by a fusion node. This pattern combines data from different sensors to create a more accurate representation of the environment or robot state.

### 2. Hierarchical Control
Higher-level controllers publish commands to lower-level controllers via topics. Lower-level controllers may provide feedback through separate topics. This creates a hierarchy where high-level decisions are refined into low-level actions.

### 3. Behavior Trees
Service calls are often used to activate specific behaviors in a behavior tree architecture. Each behavior is a service that can be called by a higher-level decision maker.

### 4. Distributed Control
Different subsystems (e.g., walking, manipulation, vision) communicate through topics to coordinate their activities. Each subsystem can operate independently while staying informed about the overall robot state.

## Best Practices for Communication Design

### Choosing the Right Pattern
- Use **topics** for continuous data streams (sensors, state information, commands)
- Use **services** for discrete actions that require confirmation (configuration, activation)
- Use **actions** for tasks that take time to complete (navigation, manipulation)

### Naming Conventions
- Use descriptive names that indicate the content and purpose
- Follow the convention: `/namespace/subsystem/data_type`
- Examples: `/robot1/joint_states`, `/vision/detected_objects`, `/control/cmd_vel`

### Data Design
- Keep messages efficient and focused
- Use appropriate message types from standard packages when possible
- Consider bandwidth and processing requirements

## Summary

ROS 2 communication patterns provide the foundation for distributed robotic systems. By choosing the appropriate pattern for each interaction and designing clear interfaces between components, you can create robust and maintainable humanoid robot systems.

## Next Steps

Now that you understand the communication patterns in ROS 2, continue to the next chapter to learn how to implement these concepts using Python and how to model humanoid robot structures with URDF.