# Assessment: Introduction to ROS 2

## Learning Objectives
After completing this chapter, students should be able to:
- Explain what ROS 2 is and its purpose in robotics
- Identify scenarios where distributed control is necessary
- Articulate why middleware is essential for robotic systems

## Questions

### Question 1: What is ROS 2?
**Multiple Choice**
Which of the following best describes ROS 2?
A) A programming language for robotics
B) A middleware framework for robot communication and coordination
C) A specific type of robot hardware
D) An operating system for robots

**Correct Answer:** B

### Question 2: Why do robots need middleware?
**Short Answer**
Explain why middleware is important for robotic systems and provide at least one example of a scenario where it would be necessary.

### Question 3: Distributed Control Concepts
**Scenario-Based**
You are designing a humanoid robot with multiple subsystems: walking control, arm movement, vision processing, and speech recognition. Explain why distributed control would be beneficial for this robot and how middleware facilitates this approach.

## Answers

### Question 1: B
ROS 2 (Robot Operating System 2) is a middleware framework that provides services designed for a heterogeneous computer cluster, including hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### Question 2:
Middleware is important for robotic systems because it:
- Enables decoupled communication between different robot components
- Allows for flexible system architecture where components can be developed and tested independently
- Provides standardized interfaces for communication
- Facilitates distributed processing across multiple computers or nodes

Example scenario: A mobile robot with separate computers for perception (cameras, LIDAR), control (motors), and high-level decision making. Middleware allows these systems to communicate seamlessly.

### Question 3:
Distributed control is beneficial because:
- Different subsystems can run at different frequencies (e.g., control loops vs. perception)
- Failure of one subsystem doesn't necessarily stop the entire robot
- Multiple developers can work on different subsystems simultaneously
- Computational load can be distributed across multiple processors

Middleware facilitates this by providing standardized communication patterns (publish/subscribe, service calls) that allow these subsystems to interact without tight coupling.