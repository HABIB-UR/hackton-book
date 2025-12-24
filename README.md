# AI-Driven Book + Embedded RAG Chatbot

This repository contains an AI-driven book with an embedded RAG chatbot, focusing on robotics and AI education.

## ROS 2 Nervous System Module

The ROS 2 Nervous System module provides educational content about ROS 2 for humanoid robot control using Docusaurus. The book includes three chapters covering:

1. Introduction to ROS 2 - Core concepts and why robots need middleware
2. Nodes, Topics, Services - Communication patterns in ROS 2
3. Python + URDF - Practical implementation with rclpy and robot modeling

### Getting Started

To run the documentation locally:

```bash
cd docs
npm install
npm run start
```

The documentation will be available at `http://localhost:3000`.

### Structure

- `docs/` - Docusaurus-based documentation
- `docs/module-1/` - ROS 2 educational content
- `docs/module-2/` - Digital Twin Simulation (Gazebo & Unity)
- `docs/module-3/` - AI-Robot Brain (NVIDIA Isaac™)
- `docs/module-4/` - Vision-Language-Action (VLA) for Autonomous Humanoids
- `docs/examples/` - Sample Python and URDF files
- `specs/` - Specification files for the project