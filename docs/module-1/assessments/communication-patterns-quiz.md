# Assessment: ROS 2 Communication Patterns

## Learning Objectives
After completing this chapter, students should be able to:
- Identify different communication patterns in ROS 2 (topics, services, actions)
- Design appropriate communication systems using nodes, topics, and services
- Explain how signals flow through a humanoid robot control system
- Implement publisher-subscriber and service-client patterns

## Questions

### Question 1: Communication Patterns
**Multiple Choice**
Which ROS 2 communication pattern is best suited for continuous sensor data like camera images or LIDAR scans?
A) Services
B) Actions
C) Topics (Publish/Subscribe)
D) Parameters

**Correct Answer:** C

### Question 2: Publisher-Subscriber Pattern
**Scenario-Based**
You are designing a humanoid robot's walking control system. The IMU sensor publishes orientation data at 100Hz, and the walking controller subscribes to this data to maintain balance. The walking controller also publishes desired joint positions to the motor controllers at 50Hz. Draw a simple diagram showing the nodes, topics, and direction of data flow.

### Question 3: Service vs Topic
**Short Answer**
Explain when you would use a service call instead of a topic for communication between ROS 2 nodes. Provide an example related to humanoid robot control.

## Answers

### Question 1: C
Topics (Publish/Subscribe) are ideal for continuous data streams like sensor readings. The publisher sends data at regular intervals, and any number of subscribers can receive the data without affecting the publisher.

### Question 2:
The system would include:
- IMU Sensor Node (publisher) → topic: `/imu/data` → Walking Controller Node (subscriber)
- Walking Controller Node (publisher) → topic: `/joint_commands` → Motor Controller Node (subscriber)

This represents a continuous flow of sensor data and control commands.

### Question 3:
Use service calls for request-response interactions where:
- You need a specific response to a specific request
- The interaction is synchronous (request waits for response)
- The action is transactional (e.g., "move to position X and tell me when done")

Example: Requesting the robot to perform a specific action like "stand up" or "walk forward 1 meter" where you need confirmation of completion.