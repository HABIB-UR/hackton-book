# ROS 2 Examples

This directory contains example code and configuration files referenced in the ROS 2 Nervous System educational material.

## Python Examples

- `simple_publisher.py`: Basic publisher node example
- `simple_subscriber.py`: Basic subscriber node example
- `simple_service.py`: Basic service server example

## URDF Examples

- `simple_humanoid.urdf`: Complete humanoid robot model with torso, head, arms, and legs

## Running the Examples

To run the Python examples, make sure you have ROS 2 installed and sourced:

```bash
# Terminal 1 - Run the publisher
python3 simple_publisher.py

# Terminal 2 - Run the subscriber
python3 simple_subscriber.py
```

To visualize the URDF model:

```bash
# Install rviz2 if not already installed
# Then run:
ros2 run rviz2 rviz2

# In RViz, add a RobotModel display and load the URDF file
```