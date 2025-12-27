---
sidebar_position: 1
---

# Introduction to ROS 2

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not actually an operating system, but rather a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms and environments.

ROS 2 is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 and to provide a more robust, secure, and scalable framework for robotics development. It builds upon the success of ROS 1 while incorporating lessons learned from years of use in research, academia, and industry.

### Key Features of ROS 2

- **Middleware**: Based on DDS (Data Distribution Service) for communication
- **Real-time support**: Enhanced capabilities for real-time systems
- **Security**: Built-in security features for safe robot operation
- **Cross-platform**: Runs on various operating systems including Linux, Windows, and macOS
- **Docker-friendly**: Better support for containerization

## Why Robots Need Middleware

Robots are complex systems that typically involve multiple sensors, actuators, and processing units working together to achieve a goal. As robots become more sophisticated, they require:

- **Multiple software components** to handle different aspects of robot operation
- **Communication** between these components in a reliable and efficient manner
- **Coordination** of different subsystems to work together seamlessly
- **Flexibility** to add or remove components as needed
- **Scalability** to handle increasing complexity

### The Middleware Solution

Middleware acts as a "software bus" that connects different parts of a robot system. It handles the complexities of communication, allowing developers to focus on implementing robot behaviors rather than managing data transmission between components.

Without middleware, developers would need to implement custom communication protocols for every pair of components that need to interact, leading to tightly coupled, inflexible, and hard-to-maintain systems.

## Core Idea of Distributed Control

Distributed control is a fundamental concept in modern robotics that recognizes that a robot's functions can be divided among multiple processing units that communicate with each other rather than being controlled by a single central processor.

### Benefits of Distributed Control

1. **Modularity**: Different subsystems can be developed, tested, and maintained independently
2. **Scalability**: New components can be added without disrupting existing functionality
3. **Fault Tolerance**: Failure in one component doesn't necessarily stop the entire system
4. **Performance**: Different components can run at different frequencies optimized for their specific tasks
5. **Team Development**: Multiple developers can work on different subsystems simultaneously

### How Distributed Control Works in ROS 2

In ROS 2, distributed control is achieved through:

- **Nodes**: Independent processes that perform computation
- **Topics**: Communication channels for data streams
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous goal-oriented communication

Each node can run on the same computer or be distributed across multiple computers, connected via standard network protocols. This allows for complex robot systems where different nodes handle perception, planning, control, and other functions.

![Distributed Control in ROS 2](/images/intro-ros2/distributed-control.svg)

The diagram above illustrates how different nodes (Perception, Planning, Control) communicate through the ROS 2 middleware, which handles the complexities of message passing and coordination between components.

## ROS 2 Architecture Overview

ROS 2 follows a distributed computing architecture where nodes communicate with each other through a publish-subscribe model, service calls, and action communication. This architecture enables:

- **Loose Coupling**: Nodes don't need to know about each other directly
- **Language Independence**: Nodes can be written in different programming languages
- **Process Isolation**: Nodes run as separate processes, improving robustness
- **Network Transparency**: Communication works the same whether nodes are on the same machine or distributed across a network

### Key Concepts

- **Packages**: Organizational units that contain nodes, libraries, and other resources
- **Workspaces**: Directories where packages are built and organized
- **Launch files**: Configuration files that start multiple nodes together
- **Parameters**: Configuration values that can be set at runtime

## Summary

ROS 2 provides the middleware and tools necessary to build complex robotic systems using distributed control principles. By separating concerns and providing standardized communication patterns, ROS 2 enables developers to focus on creating innovative robot behaviors rather than managing the complexities of inter-component communication.

## Next Steps

Now that you understand the fundamentals of ROS 2 and distributed control, continue to the next chapter to explore how ROS 2 nodes communicate with each other through topics, services, and actions, and how these communication patterns apply to humanoid robot control systems.