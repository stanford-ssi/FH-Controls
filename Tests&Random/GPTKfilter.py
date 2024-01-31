import numpy as np
import matplotlib.pyplot as plt

def kalman_filter_3d(imu_data, initial_state, process_variance, measurement_variance):
    # Initialization
    state = np.array(initial_state)
    P = np.eye(6)  # Initial covariance matrix
    F = np.block([[np.eye(3), np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])  # State transition matrix
    H = np.block([[np.zeros((3, 3)), np.eye(3)]])

    # Process and measurement noise
    Q = np.block([[process_variance * np.eye(3), np.zeros((3, 3))],
                  [np.zeros((3, 3)), process_variance * np.eye(3)]])
    R = measurement_variance * np.eye(3)

    # Lists to store estimated states
    estimated_positions = []
    estimated_velocities = []

    for measurement in imu_data:
        # Prediction step
        state = np.dot(F, state)
        P = np.dot(np.dot(F, P), F.T) + Q

        # Update step
        y = measurement - np.dot(H, state)
        S = np.dot(np.dot(H, P), H.T) + R
        K = np.dot(np.dot(P, H.T), np.linalg.inv(S))

        state = state + np.dot(K, y)
        P = P - np.dot(np.dot(K, H), P)

        # Store estimated states
        estimated_positions.append(state[:3])
        estimated_velocities.append(state[3:])

    return np.array(estimated_positions), np.array(estimated_velocities)

# Generate synthetic 3D IMU data
np.random.seed(42)
time = np.arange(0, 10, 0.1)
true_acceleration = np.column_stack((np.sin(2 * np.pi * 0.5 * time),
                                     0.5 * np.sin(2 * np.pi * 0.2 * time),
                                     np.cos(2 * np.pi * 0.3 * time)))  # True acceleration
noisy_acceleration = true_acceleration + np.random.normal(0, 0.1, size=true_acceleration.shape)  # Add noise

# Run 3D Kalman filter
initial_state_estimate = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
process_variance_estimate = 0.01
measurement_variance_estimate = 0.1

estimated_positions, estimated_velocities = kalman_filter_3d(
    imu_data=noisy_acceleration,
    initial_state=initial_state_estimate,
    process_variance=process_variance_estimate,
    measurement_variance=measurement_variance_estimate
)

# Plot results
plt.figure(figsize=(15, 10))
for i in range(3):
    plt.subplot(3, 1, i + 1)
    plt.plot(time, true_acceleration[:, i], label=f'True Acceleration {["X", "Y", "Z"][i]}')
    plt.plot(time, noisy_acceleration[:, i], label=f'Noisy Acceleration {["X", "Y", "Z"][i]}', linestyle='dashed')
    plt.plot(time, estimated_positions[:, i], label=f'Estimated Position {["X", "Y", "Z"][i]}', linestyle='dotted')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

plt.tight_layout()
plt.show()