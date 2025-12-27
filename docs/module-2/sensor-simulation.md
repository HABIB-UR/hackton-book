---
sidebar_position: 3
---

# Sensor Simulation

## What is Sensor Simulation?

Sensor simulation encompasses the modeling and generation of realistic sensor data for humanoid robots in virtual environments. This chapter covers LiDAR, depth, IMU, and other sensor simulation pipelines that enable robots to perceive and interact with their virtual surroundings.

## Learning Objectives

By the end of this chapter, you will understand:
- How to simulate LiDAR sensors and generate realistic point cloud data
- How to implement depth sensor simulation for RGB-D camera systems
- How to model IMU sensors for inertial measurement and orientation
- How to create sensor fusion pipelines for integrated perception
- How to validate and calibrate simulated sensors

## Table of Contents
- [LiDAR Simulation](#lidar-simulation)
- [Depth Sensor Simulation](#depth-sensor-simulation)
- [IMU Simulation](#imu-simulation)
- [Sensor Fusion Concepts](#sensor-fusion-concepts)
- [Sensor Calibration](#sensor-calibration)
- [Practical Examples](#practical-examples)
- [Assessment](#assessment)

## LiDAR Simulation

LiDAR (Light Detection and Ranging) simulation generates 3D point cloud data that represents the environment from the robot's perspective. This data is crucial for navigation, mapping, and obstacle detection in humanoid robot applications.

### LiDAR Physics and Modeling

LiDAR sensors work by emitting laser pulses and measuring the time it takes for the light to return after reflecting off objects. Simulating this process requires understanding the physical properties of light and how it interacts with different surfaces.

#### Key Parameters for LiDAR Simulation

- **Range**: Maximum distance the sensor can detect objects
- **Field of View (FOV)**: Angular coverage of the sensor (horizontal and vertical)
- **Resolution**: Angular resolution of the sensor (degrees per measurement)
- **Update Rate**: How frequently the sensor provides new measurements
- **Accuracy**: Precision of distance measurements
- **Intensity**: Reflectance properties of detected surfaces

### LiDAR Simulation Techniques

#### Raycasting Approach

The most common approach for LiDAR simulation uses raycasting to determine distances to objects in the environment:

1. Emit rays from the sensor in the pattern of the real LiDAR
2. Calculate intersection points with environment geometry
3. Record distance, intensity, and other properties
4. Generate point cloud data in standard formats (e.g., PCD, PLY)

#### Performance Considerations

- **Ray Count**: Balance accuracy with computational efficiency
- **Update Frequency**: Match real sensor update rates
- **Level of Detail**: Adjust based on distance and importance
- **Occlusion Handling**: Properly handle partial obstructions

## Depth Sensor Simulation

Depth sensors provide 2.5D information about the environment, capturing both color (RGB) and distance (depth) information. This data is essential for object recognition, manipulation, and scene understanding.

### RGB-D Simulation

RGB-D sensors combine color and depth information in a single sensor package. Simulation must account for both modalities and their interdependencies.

#### Depth Map Generation

Depth maps represent distance information as grayscale values, where darker pixels represent closer objects and lighter pixels represent farther objects.

#### Noise Modeling

Real depth sensors have various sources of noise and error:
- **Quantization Noise**: Discrete depth measurement steps
- **Gaussian Noise**: Random measurement errors
- **Dropout Regions**: Areas where depth cannot be measured
- **Multi-path Interference**: Errors from reflective surfaces

### Depth Camera Models

#### Pinhole Camera Model

The pinhole camera model describes how 3D points are projected onto a 2D image plane:
- **Focal Length**: Distance from optical center to image plane
- **Principal Point**: Center of the image in pixel coordinates
- **Distortion Parameters**: Corrections for lens distortion

#### Depth Accuracy Models

Depth accuracy typically varies with distance and viewing angle:
- **Systematic Errors**: Consistent biases in measurements
- **Random Errors**: Stochastic variations in measurements
- **Boundary Effects**: Inaccuracies at object boundaries

## IMU Simulation

Inertial Measurement Units (IMUs) provide information about a robot's orientation, angular velocity, and linear acceleration. IMU simulation is critical for humanoid robot balance, navigation, and motion control.

### IMU Components

An IMU typically contains three types of sensors:

#### Accelerometer
- Measures linear acceleration in three axes
- Used to determine orientation relative to gravity
- Sensitive to both gravity and motion acceleration

#### Gyroscope
- Measures angular velocity around three axes
- Provides information about rotation rates
- Subject to drift over time

#### Magnetometer
- Measures magnetic field direction
- Provides absolute orientation reference
- Susceptible to magnetic interference

### IMU Noise and Error Modeling

#### Bias
- **Constant bias**: Consistent offset in measurements
- **Temperature drift**: Bias changes with temperature
- **Time drift**: Bias changes over time

#### Noise
- **White noise**: Random fluctuations in measurements
- **Random walk**: Slowly varying bias components
- **Quantization noise**: Discrete measurement steps

#### Cross-axis Sensitivity
- Measurements on one axis may be affected by motion on other axes
- Requires calibration matrix to correct

## Sensor Fusion Concepts

Sensor fusion combines data from multiple sensors to create a more accurate and robust perception of the environment than any single sensor could provide.

### Kalman Filtering

Kalman filters provide an optimal way to combine sensor measurements with different characteristics:
- **Prediction**: Use motion models to predict state
- **Update**: Incorporate new sensor measurements
- **Covariance**: Track uncertainty in estimates

### Particle Filtering

Particle filters are useful for non-linear, non-Gaussian systems:
- **Particles**: Represent possible states of the system
- **Weights**: Reflect likelihood of each particle
- **Resampling**: Focus on more likely particles over time

### Multi-Sensor Integration

#### Data Association
- Match sensor measurements to known landmarks
- Handle false positives and missed detections
- Maintain consistent world representation

#### Temporal Synchronization
- Align measurements from sensors with different update rates
- Account for communication delays
- Interpolate measurements to common time stamps

## Sensor Calibration

Calibration ensures that simulated sensors accurately reflect the characteristics of their real-world counterparts.

### Intrinsic Calibration

Intrinsic calibration parameters describe the internal characteristics of a sensor:
- **Camera intrinsic parameters**: Focal length, principal point, distortion
- **LiDAR parameters**: Range accuracy, angular resolution, beam divergence
- **IMU parameters**: Scale factors, bias terms, cross-axis sensitivity

### Extrinsic Calibration

Extrinsic calibration defines the position and orientation of sensors relative to the robot:
- **Position**: 3D coordinates of sensor relative to robot frame
- **Orientation**: Rotation of sensor relative to robot frame
- **Temporal offset**: Time delay between sensors

## Practical Examples

This section provides practical examples of implementing sensor simulation for humanoid robots.

### Example 1: LiDAR Simulation in Unity

```csharp
using UnityEngine;
using System.Collections.Generic;

public class LIDARSimulator : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    public int horizontalRays = 360;
    public int verticalRays = 16;
    public float minAngle = -15f;
    public float maxAngle = 15f;
    public float maxRange = 50f;
    public LayerMask detectionMask;

    [Header("Noise Parameters")]
    public float rangeNoise = 0.01f; // 1cm standard deviation

    public struct PointCloudPoint
    {
        public Vector3 position;
        public float intensity;
        public float range;
    }

    public List<PointCloudPoint> GeneratePointCloud()
    {
        List<PointCloudPoint> points = new List<PointCloudPoint>();

        float horizontalStep = 360f / horizontalRays;
        float verticalStep = (maxAngle - minAngle) / verticalRays;

        for (int h = 0; h < horizontalRays; h++)
        {
            for (int v = 0; v < verticalRays; v++)
            {
                float hAngle = h * horizontalStep;
                float vAngle = minAngle + v * verticalStep;

                // Convert angles to direction vector
                float hRad = hAngle * Mathf.Deg2Rad;
                float vRad = vAngle * Mathf.Deg2Rad;

                Vector3 direction = new Vector3(
                    Mathf.Cos(vRad) * Mathf.Sin(hRad),
                    Mathf.Cos(vRad) * Mathf.Cos(hRad),
                    Mathf.Sin(vRad)
                );

                direction = transform.TransformDirection(direction);

                // Raycast to find intersection
                RaycastHit hit;
                if (Physics.Raycast(transform.position, direction, out hit, maxRange, detectionMask))
                {
                    float range = hit.distance;

                    // Add noise to range measurement
                    range += Random.Range(-rangeNoise, rangeNoise);

                    if (range <= maxRange)
                    {
                        PointCloudPoint point = new PointCloudPoint
                        {
                            position = hit.point,
                            intensity = 1.0f, // Simplified intensity
                            range = range
                        };
                        points.Add(point);
                    }
                }
            }
        }

        return points;
    }
}
```

### Example 2: Depth Camera Simulation

```python
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class DepthCameraSimulator:
    def __init__(self, width=640, height=480, fov=60):
        self.width = width
        self.height = height
        self.fov = fov

        # Calculate intrinsic parameters
        self.fx = self.width / (2 * np.tan(np.radians(fov/2)))
        self.fy = self.fx  # Assume square pixels
        self.cx = self.width / 2
        self.cy = self.height / 2

        self.K = np.array([[self.fx, 0, self.cx],
                          [0, self.fy, self.cy],
                          [0, 0, 1]])

    def generate_depth_map(self, point_cloud, camera_pose):
        """
        Generate depth map from 3D point cloud
        camera_pose: (position, rotation_quaternion)
        """
        pos, quat = camera_pose
        rot = R.from_quat(quat).as_matrix()

        # Transform points to camera coordinate system
        R_world_to_cam = rot.T
        t_world_to_cam = -R_world_to_cam @ pos

        # Transform points
        points_hom = np.hstack([point_cloud, np.ones((point_cloud.shape[0], 1))])
        points_cam = (R_world_to_cam @ point_cloud.T + t_world_to_cam[:, np.newaxis]).T

        # Project to image plane
        points_image = (self.K @ points_cam.T).T
        points_image = points_image[:, :2] / points_image[:, 2:3]  # Normalize by z

        # Create depth map
        depth_map = np.zeros((self.height, self.width))

        # Filter points within image bounds
        valid = (points_image[:, 0] >= 0) & (points_image[:, 0] < self.width) & \
                (points_image[:, 1] >= 0) & (points_image[:, 1] < self.height) & \
                (points_cam[:, 2] > 0)  # In front of camera

        valid_points = points_image[valid].astype(int)
        valid_depths = points_cam[valid, 2]

        for i in range(len(valid_points)):
            x, y = valid_points[i]
            if 0 <= x < self.width and 0 <= y < self.height:
                if depth_map[y, x] == 0 or valid_depths[i] < depth_map[y, x]:
                    depth_map[y, x] = valid_depths[i]

        return depth_map

    def add_noise(self, depth_map, noise_level=0.01):
        """Add realistic noise to depth map"""
        # Add Gaussian noise
        noise = np.random.normal(0, noise_level, depth_map.shape)

        # Add depth-dependent noise (farther objects have more noise)
        depth_dependent_noise = noise_level * depth_map * 0.1
        total_noise = noise + np.random.normal(0, depth_dependent_noise)

        noisy_depth = depth_map + total_noise
        return np.clip(noisy_depth, 0, np.inf)

# Example usage
simulator = DepthCameraSimulator()
# Assuming we have a point cloud from the environment
# depth_map = simulator.generate_depth_map(point_cloud, camera_pose)
# noisy_depth_map = simulator.add_noise(depth_map)
```

### Example 3: IMU Simulation with Noise Modeling

```python
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

class IMUSimulator:
    def __init__(self, sample_rate=100):
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate

        # Noise parameters based on typical IMU specs (e.g., MPU-6050)
        self.accel_noise_density = 0.0025  # m/s^2/sqrt(Hz)
        self.accel_random_walk = 0.0005   # m/s^2/sqrt(s)

        self.gyro_noise_density = 0.00033 # rad/s/sqrt(Hz)
        self.gyro_random_walk = 0.00003  # rad/s/sqrt(s)

        # Bias parameters
        self.accel_bias_drift = 0.0001    # m/s^2/sqrt(s)
        self.gyro_bias_drift = 0.00001   # rad/s/sqrt(s)

        # Initialize bias states
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

        # Initialize true values (these would come from simulation)
        self.true_accel = np.zeros(3)
        self.true_gyro = np.zeros(3)

    def update_true_values(self, true_accel, true_gyro):
        """Update true acceleration and angular velocity values"""
        self.true_accel = true_accel
        self.true_gyro = true_gyro

    def simulate_measurement(self):
        """Generate simulated IMU measurement with noise and bias"""
        # Generate noise components
        accel_white_noise = np.random.normal(0, self.accel_noise_density / np.sqrt(self.dt), 3)
        gyro_white_noise = np.random.normal(0, self.gyro_noise_density / np.sqrt(self.dt), 3)

        # Update bias random walks
        self.accel_bias += np.random.normal(0, self.accel_bias_drift * np.sqrt(self.dt), 3)
        self.gyro_bias += np.random.normal(0, self.gyro_bias_drift * np.sqrt(self.dt), 3)

        # Apply noise and bias to true values
        measured_accel = self.true_accel + self.accel_bias + accel_white_noise
        measured_gyro = self.true_gyro + self.gyro_bias + gyro_white_noise

        # Add gravity to accelerometer (IMU measures proper acceleration)
        # Gravity vector in IMU frame (assuming IMU is aligned with world frame initially)
        gravity = np.array([0, 0, -9.81])
        measured_accel += gravity

        return {
            'accelerometer': measured_accel,
            'gyroscope': measured_gyro,
            'timestamp': self.dt
        }

# Example usage
imu_sim = IMUSimulator(sample_rate=100)

# In simulation loop:
# imu_sim.update_true_values(true_accel_from_physics, true_gyro_from_physics)
# measurement = imu_sim.simulate_measurement()
```

## Assessment

Complete the quiz to test your understanding of sensor simulation concepts.

[Quiz Link](./assessments/sensor-quiz.md)

## Summary

This chapter covered sensor simulation for humanoid robots, including LiDAR, depth, and IMU simulation pipelines. You learned about the physics of each sensor type, noise modeling, calibration techniques, and sensor fusion concepts. These simulation techniques are essential for creating realistic training environments where humanoid robots can develop perception capabilities before real-world deployment.

## Next Steps

Review all three chapters in Module 2 to solidify your understanding of digital twin simulation concepts for humanoid robots.

[Previous: Unity Interaction Model](./unity-interaction-model.md)
[Previous: Gazebo Physics Simulation](./gazebo-physics-simulation.md)

## Related Topics

- [ROS 2 Nervous System](../module-1/intro-to-ros2.md) - Foundation concepts for robot control and communication