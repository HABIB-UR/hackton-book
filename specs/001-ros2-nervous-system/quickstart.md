# Quickstart: ROS 2 Nervous System for Humanoid Robots

## Prerequisites

1. **System Requirements**
   - Ubuntu 22.04 LTS or Windows 10/11 with WSL2
   - Node.js 18+ and npm/yarn
   - Python 3.8+
   - ROS 2 Humble Hawksbill installed

2. **Install Docusaurus**
   ```bash
   npm init docusaurus@latest docs classic
   ```

3. **Setup Python Environment**
   ```bash
   python3 -m pip install rclpy
   ```

## Getting Started

1. **Clone the Repository**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install Dependencies**
   ```bash
   cd docs
   npm install
   ```

3. **Run Local Development Server**
   ```bash
   npm run start
   ```

4. **Access the Documentation**
   - Open `http://localhost:3000` in your browser
   - Navigate to Module 1 to access the ROS 2 content

## Chapter Structure

The ROS 2 Nervous System module contains three chapters:

1. **Introduction to ROS 2** - Core concepts and why robots need middleware
2. **Nodes, Topics, Services** - Communication patterns in ROS 2
3. **Python + URDF** - Practical implementation with rclpy and robot modeling

## Running Examples

Each chapter includes practical examples that can be run in your ROS 2 environment:

1. Navigate to the example directory
2. Source your ROS 2 environment: `source /opt/ros/humble/setup.bash`
3. Run the example: `python3 example.py`

## Building for Production

```bash
npm run build
```

The built site will be available in the `build/` directory and can be served statically.