---
sidebar_position: 3
---

# Python + URDF

## Introduction to rclpy

rclpy is the Python client library for ROS 2. It provides a Python API that allows you to create ROS 2 nodes, publish and subscribe to topics, make service calls, and more. Python is an excellent choice for ROS 2 development due to its simplicity, readability, and extensive ecosystem of scientific and robotics libraries.

### Installing rclpy

rclpy is included with the ROS 2 installation. To use it in your Python projects, simply import it:

```python
import rclpy
from rclpy.node import Node
```

## Creating Your First ROS 2 Node in Python

![ROS 2 Python Node Structure](/images/python-urdf/python-node-structure.svg)

A ROS 2 node in Python is a class that inherits from `rclpy.node.Node`. Here's the basic structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Node initialization code goes here
        self.get_logger().info('MyNode has been started')

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS communications
    node = MyNode()        # Create your node
    rclpy.spin(node)       # Keep the node running
    node.destroy_node()    # Clean up
    rclpy.shutdown()       # Shutdown ROS communications

if __name__ == '__main__':
    main()
```

### Key Components Explained:

1. **rclpy.init()**: Initializes the ROS 2 client library
2. **Node**: The base class for all ROS 2 nodes
3. **rclpy.spin()**: Keeps the node alive and processes callbacks
4. **Node destruction**: Properly cleans up resources

## Publisher Implementation

A publisher node sends messages to a topic. Here's how to create one:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscriber Implementation

A subscriber node receives messages from a topic:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Service Implementation

Services provide request-response communication:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Understanding URDF (Unified Robot Description Format)

URDF (Unified Robot Description Format) is an XML format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links, joints, and how they connect to form the robot structure.

### Basic URDF Structure

A URDF file consists of:

1. **Links**: Rigid bodies that make up the robot
2. **Joints**: Connections between links with specific degrees of freedom
3. **Materials**: Visual properties like color and texture
4. **Inertial properties**: Mass, center of mass, and inertia for physics simulation

### Simple URDF Example

Here's a basic URDF for a simple robot with a base and a single link:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Second link -->
  <link name="link2">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Joint connecting the links -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link2"/>
    <origin xyz="0.3 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1000.0" velocity="0.5"/>
  </joint>
</robot>
```

## URDF Components Explained

### Links
- Represent rigid bodies in the robot
- Each link can have visual, collision, and inertial properties
- The first link is typically the "base_link" and serves as the reference frame

### Joints
- Define how links connect to each other
- Types include: revolute (rotational), prismatic (linear), fixed, continuous, etc.
- Specify parent and child links, position, and motion limits

### Visual Properties
- Define how the robot appears in simulation and visualization tools
- Include geometry (box, cylinder, sphere, mesh) and materials

### Collision Properties
- Define the shape used for collision detection
- May be simpler than visual geometry for performance

## URDF for Humanoid Robots

Humanoid robots require more complex URDF structures with multiple limbs and joints. A typical humanoid URDF includes:

- **Torso**: The main body with head, arms, and legs attached
- **Head**: With sensors like cameras
- **Arms**: With shoulders, elbows, wrists, and hands
- **Legs**: With hips, knees, ankles, and feet
- **Joints**: All configured with appropriate degrees of freedom

### Example Humanoid Structure
```
base_link (torso)
├── head_link
├── left_upper_arm_link
│   └── left_lower_arm_link
│       └── left_hand_link
├── right_upper_arm_link
│   └── right_lower_arm_link
│       └── right_hand_link
├── left_upper_leg_link
│   └── left_lower_leg_link
│       └── left_foot_link
└── right_upper_leg_link
    └── right_lower_leg_link
        └── right_foot_link
```

## Creating a Simple Humanoid URDF

Here's a simplified URDF for a basic humanoid robot:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="skin">
        <color rgba="0.8 0.6 0.4 1"/>
      </material>
    </visual>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.45" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
</robot>
```

## Using URDF with ROS 2

URDF files are typically loaded into ROS 2 using the `robot_state_publisher` package, which reads the URDF and publishes the robot's joint states and transforms.

```bash
# Launch the robot state publisher with your URDF
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="..."
```

## Best Practices

### Python Development
- Use proper error handling and logging
- Follow ROS 2 naming conventions
- Use appropriate QoS (Quality of Service) profiles
- Implement proper cleanup in node destruction

### URDF Development
- Start simple and add complexity gradually
- Use consistent naming conventions
- Include proper inertial properties for simulation
- Validate URDF files using tools like `check_urdf`

## Summary

This chapter covered the fundamentals of implementing ROS 2 concepts in Python using rclpy and modeling humanoid robots with URDF. You now have the knowledge to create nodes that communicate through topics and services, and to define robot structures that can be used in simulation and visualization. These skills form the foundation for developing complex humanoid robot applications.

## Next Steps

With the fundamentals of ROS 2 covered, you can now explore more advanced topics such as:
- Advanced ROS 2 concepts like Actions and Parameters
- Robot simulation with Gazebo
- Navigation and path planning
- Perception systems and computer vision
- Advanced control algorithms for humanoid robots