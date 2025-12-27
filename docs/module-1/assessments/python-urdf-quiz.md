# Assessment: Python Implementation and URDF

## Learning Objectives
After completing this chapter, students should be able to:
- Create basic ROS 2 nodes in Python using rclpy
- Create simple URDF files that model humanoid robot structures
- Implement publisher and subscriber nodes in Python
- Understand the structure of URDF files and their components

## Questions

### Question 1: rclpy Basics
**Multiple Choice**
Which of the following is the correct way to initialize a ROS 2 node in Python using rclpy?
A) `rclpy.init_node('my_node')`
B) `node = rclpy.create_node('my_node')`
C) `rclpy.init(); node = Node('my_node')`
D) `node = rclpy.Node('my_node')`

**Correct Answer:** C

### Question 2: URDF Structure
**Short Answer**
List the three main types of elements that must be defined in a URDF file for a simple robot with two joints.

### Question 3: Python Node Implementation
**Code Exercise**
Write a simple Python ROS 2 publisher node that publishes a string message "Hello, ROS 2!" to a topic called "greeting" at 1 Hz.

## Answers

### Question 1: C
The correct way is to first initialize rclpy with `rclpy.init()`, then create a node using `Node('node_name')`. This follows the ROS 2 pattern where rclpy must be initialized before creating nodes.

### Question 2:
The three main types of elements in a URDF file are:
1. **Links**: Represent rigid bodies (e.g., robot parts like arms, base)
2. **Joints**: Define connections between links with specific degrees of freedom
3. **Materials**: Define visual properties like color (optional but commonly used)

### Question 3:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GreetingPublisher(Node):
    def __init__(self):
        super().__init__('greeting_publisher')
        self.publisher = self.create_publisher(String, 'greeting', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    greeting_publisher = GreetingPublisher()
    rclpy.spin(greeting_publisher)
    greeting_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```