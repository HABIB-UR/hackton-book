"""
Sensor Fusion Example for Humanoid Robot
This example demonstrates how to combine data from multiple sensors
to create a more accurate estimate of the robot's state.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

class ExtendedKalmanFilter:
    """
    Extended Kalman Filter for fusing IMU and visual odometry data
    """
    def __init__(self, state_dim=6, measurement_dim=6):
        # State: [x, y, z, roll, pitch, yaw]
        self.state_dim = state_dim
        self.measurement_dim = measurement_dim

        # State vector [position, orientation]
        self.x = np.zeros(state_dim)

        # Covariance matrix
        self.P = np.eye(state_dim) * 1000  # High initial uncertainty

        # Process noise
        self.Q = np.eye(state_dim) * 0.1

        # Measurement noise
        self.R_imu = np.eye(3) * 0.01    # Low noise for IMU orientation
        self.R_vo = np.eye(3) * 0.5      # Higher noise for visual odometry position

    def predict(self, dt, control_input=None):
        """
        Prediction step: predict next state based on motion model
        """
        # Simplified motion model - in practice, this would use control inputs
        # For now, assume constant velocity model
        F = np.eye(self.state_dim)  # Jacobian of motion model
        # Add time-based state transition
        self.x = self.x  # In this simple model, state remains the same without control

        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

    def update_imu(self, orientation_measurement):
        """
        Update step for IMU data (orientation only)
        """
        # Measurement matrix for orientation (last 3 elements of state)
        H = np.zeros((3, self.state_dim))
        H[0, 3] = 1  # roll
        H[1, 4] = 1  # pitch
        H[2, 5] = 1  # yaw

        # Innovation
        y = orientation_measurement - self.x[3:6]  # Difference between measurement and prediction

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_imu

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ H) @ self.P

    def update_visual_odometry(self, position_measurement):
        """
        Update step for visual odometry data (position only)
        """
        # Measurement matrix for position (first 3 elements of state)
        H = np.zeros((3, self.state_dim))
        H[0, 0] = 1  # x
        H[1, 1] = 1  # y
        H[2, 2] = 1  # z

        # Innovation
        y = position_measurement - self.x[0:3]  # Difference between measurement and prediction

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_vo

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(self.state_dim)
        self.P = (I - K @ H) @ self.P


class SensorSimulator:
    """
    Simulates realistic sensor data for a humanoid robot
    """
    def __init__(self):
        self.true_position = np.array([0.0, 0.0, 0.0])
        self.true_orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.velocity = np.array([0.1, 0.05, 0.0])  # m/s

        # Sensor noise parameters
        self.imu_orientation_noise = 0.01  # rad
        self.position_noise = 0.05  # m

    def update_true_state(self, dt):
        """
        Update the true state of the robot (for simulation purposes)
        """
        self.true_position += self.velocity * dt
        # Add some rotation for more realistic movement
        self.true_orientation[2] += 0.01  # Slow yaw rotation

    def simulate_imu(self):
        """
        Simulate IMU measurement with noise
        """
        # Add noise to true orientation
        noisy_orientation = self.true_orientation + np.random.normal(0, self.imu_orientation_noise, 3)
        return noisy_orientation

    def simulate_position_sensor(self):
        """
        Simulate position measurement with noise (e.g., from visual odometry)
        """
        # Add noise to true position
        noisy_position = self.true_position + np.random.normal(0, self.position_noise, 3)
        return noisy_position


def main():
    """
    Main function demonstrating sensor fusion
    """
    # Initialize filter and simulator
    ekf = ExtendedKalmanFilter()
    simulator = SensorSimulator()

    # Store results for plotting
    true_positions = []
    measured_positions = []
    fused_positions = []

    true_orientations = []
    measured_orientations = []
    fused_orientations = []

    dt = 0.1  # 10 Hz
    duration = 10  # seconds
    steps = int(duration / dt)

    print("Starting sensor fusion simulation...")

    for i in range(steps):
        # Update true state
        simulator.update_true_state(dt)

        # Get simulated measurements
        imu_measurement = simulator.simulate_imu()
        position_measurement = simulator.simulate_position_sensor()

        # Prediction step
        ekf.predict(dt)

        # Update steps
        ekf.update_imu(imu_measurement)
        ekf.update_visual_odometry(position_measurement)

        # Store data for analysis
        true_positions.append(simulator.true_position.copy())
        measured_positions.append(position_measurement.copy())
        fused_positions.append(ekf.x[0:3].copy())

        true_orientations.append(simulator.true_orientation.copy())
        measured_orientations.append(imu_measurement.copy())
        fused_orientations.append(ekf.x[3:6].copy())

        if i % 50 == 0:  # Print progress every 5 seconds
            print(f"Step {i}/{steps}: Position - True: {simulator.true_position}, Fused: {ekf.x[0:3]}")

    # Convert to numpy arrays for plotting
    true_positions = np.array(true_positions)
    measured_positions = np.array(measured_positions)
    fused_positions = np.array(fused_positions)

    # Plot results
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))

    # Position plots
    axes[0, 0].plot(true_positions[:, 0], label='True X')
    axes[0, 0].plot(measured_positions[:, 0], label='Measured X', alpha=0.7)
    axes[0, 0].plot(fused_positions[:, 0], label='Fused X', alpha=0.7)
    axes[0, 0].set_title('X Position Over Time')
    axes[0, 0].set_xlabel('Time Step')
    axes[0, 0].set_ylabel('Position (m)')
    axes[0, 0].legend()

    axes[0, 1].plot(true_positions[:, 1], label='True Y')
    axes[0, 1].plot(measured_positions[:, 1], label='Measured Y', alpha=0.7)
    axes[0, 1].plot(fused_positions[:, 1], label='Fused Y', alpha=0.7)
    axes[0, 1].set_title('Y Position Over Time')
    axes[0, 1].set_xlabel('Time Step')
    axes[0, 1].set_ylabel('Position (m)')
    axes[0, 1].legend()

    axes[0, 2].plot(true_positions[:, 2], label='True Z')
    axes[0, 2].plot(measured_positions[:, 2], label='Measured Z', alpha=0.7)
    axes[0, 2].plot(fused_positions[:, 2], label='Fused Z', alpha=0.7)
    axes[0, 2].set_title('Z Position Over Time')
    axes[0, 2].set_xlabel('Time Step')
    axes[0, 2].set_ylabel('Position (m)')
    axes[0, 2].legend()

    # Orientation plots
    axes[1, 0].plot(np.rad2deg(true_orientations[:, 0]), label='True Roll')
    axes[1, 0].plot(np.rad2deg(measured_orientations[:, 0]), label='Measured Roll', alpha=0.7)
    axes[1, 0].plot(np.rad2deg(fused_orientations[:, 0]), label='Fused Roll', alpha=0.7)
    axes[1, 0].set_title('Roll Over Time')
    axes[1, 0].set_xlabel('Time Step')
    axes[1, 0].set_ylabel('Angle (degrees)')
    axes[1, 0].legend()

    axes[1, 1].plot(np.rad2deg(true_orientations[:, 1]), label='True Pitch')
    axes[1, 1].plot(np.rad2deg(measured_orientations[:, 1]), label='Measured Pitch', alpha=0.7)
    axes[1, 1].plot(np.rad2deg(fused_orientations[:, 1]), label='Fused Pitch', alpha=0.7)
    axes[1, 1].set_title('Pitch Over Time')
    axes[1, 1].set_xlabel('Time Step')
    axes[1, 1].set_ylabel('Angle (degrees)')
    axes[1, 1].legend()

    axes[1, 2].plot(np.rad2deg(true_orientations[:, 2]), label='True Yaw')
    axes[1, 2].plot(np.rad2deg(measured_orientations[:, 2]), label='Measured Yaw', alpha=0.7)
    axes[1, 2].plot(np.rad2deg(fused_orientations[:, 2]), label='Fused Yaw', alpha=0.7)
    axes[1, 2].set_title('Yaw Over Time')
    axes[1, 2].set_xlabel('Time Step')
    axes[1, 2].set_ylabel('Angle (degrees)')
    axes[1, 2].legend()

    plt.tight_layout()
    plt.savefig('sensor_fusion_results.png', dpi=150, bbox_inches='tight')
    plt.show()

    print("Sensor fusion simulation completed!")
    print(f"Final position - True: {simulator.true_position}, Fused: {ekf.x[0:3]}")
    print(f"Final orientation - True: {np.rad2deg(simulator.true_orientation)}, Fused: {np.rad2deg(ekf.x[3:6])}")


if __name__ == "__main__":
    main()