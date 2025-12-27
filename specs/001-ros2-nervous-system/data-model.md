# Data Model: ROS 2 Nervous System for Humanoid Robots

## Entities

### ROS 2 Concepts
- **Name**: String (e.g., "Node", "Topic", "Service")
- **Definition**: Text description of the ROS 2 concept
- **Purpose**: Explanation of why the concept is important
- **Relationships**: Links to related concepts (e.g., Topics connect Publishers and Subscribers)

### Humanoid Robot Models
- **Name**: String (e.g., "Biped Robot", "Quadruped Robot")
- **Description**: Text description of the robot type
- **URDF Path**: File path to the URDF model
- **Joint Configuration**: Array of joint definitions
- **Link Configuration**: Array of link definitions

### Communication Patterns
- **Pattern Type**: Enum (Publisher-Subscriber, Service-Client, Action)
- **Message Type**: String describing the message structure
- **Frequency**: Number (Hz) for publisher-subscriber patterns
- **Use Case**: Description of when to use this pattern
- **Example Code**: Code snippet demonstrating the pattern

### Chapter Content
- **Title**: String (chapter title)
- **Content**: Markdown content of the chapter
- **Learning Objectives**: Array of learning objectives
- **Examples**: Array of code examples
- **Exercises**: Array of practice exercises
- **Dependencies**: Array of prerequisite concepts

## Relationships

- Chapter Content references ROS 2 Concepts to explain theoretical concepts
- Chapter Content includes Humanoid Robot Models for practical examples
- Communication Patterns are demonstrated in Chapter Content
- ROS 2 Concepts connect to each other to build comprehensive understanding

## Validation Rules

- All ROS 2 Concepts must have a clear, unambiguous definition
- Humanoid Robot Models must have valid URDF syntax
- Chapter Content must include at least one practical example per concept
- All code examples must be tested and verified with ROS 2 Humble Hawksbill
- Learning objectives must be measurable and achievable