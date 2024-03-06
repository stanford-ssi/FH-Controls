from ulab import numpy as np
import time

def get_readings(sensor):
    # Pull sensor readings
    mag_x, mag_y, mag_z = sensor.magnetic # uT
    acc_x, acc_y, acc_z = sensor.acceleration # m/s^2
    gyr_x, gyr_y, gyr_z = sensor.gyro # rad/s
    mag = np.array([mag_x, mag_y, mag_z])
    gyr = np.array([gyr_x, gyr_y, gyr_z])
    acc = np.array([acc_x, acc_y, acc_z])
    return acc, mag, gyr

def angles_from_mag(mag, acc):
    # Convert to unit vectors
    mag_unit = mag / np.linalg.norm(mag)
    acc_unit = acc / np.linalg.norm(acc)
    
    # Find Down, North, East directions
    D = acc_unit
    E = np.cross(D, mag_unit)
    E = E / np.linalg.norm(E)
    N = np.cross(E, D)
    N = N / np.linalg.norm(N)
    
    R = np.array([N, E, D])
    yaw = np.degrees(np.arctan2(R[2,1], R[2,2]))
    pitch = np.degrees(np.arctan2(R[2,0], np.sqrt((R[2,1] ** 2) + (R[2,2] ** 2))))
    roll = np.degrees(np.arctan2(R[1,0], R[0,0]))
    
    
    # Convert into heading, assuming flat and facing up
    # roll = np.degrees(np.arctan2(N[1], N[0]))
    # pitch = np.degrees(np.asin(N[2]))
    # yaw = 1
    
    return [pitch, yaw, roll]

def dead_reckoning(gyr, angles_gyr, dt):
    d_angles = gyr * dt
    angles_gyr = angles_gyr + d_angles
    return angles_gyr

def get_rotational_data(sensor):
    angles_mag = np.empty((0,3))
    acc, mag, gyr = get_readings(sensor)
    angles_gyr = np.array([angles_from_mag(mag, acc)])
    start_time = time.monotonic_ns()

    for i in range(10):
        acc, mag, gyr = get_readings(sensor)
        # Find Down, North East vectors
        angles_mag = np.concatenate((angles_mag, np.array([angles_from_mag(mag, acc)])), axis=0)
        
        time_now = time.monotonic_ns()
        angles_gyr = np.concatenate((angles_gyr, np.array([dead_reckoning(gyr, angles_gyr[-1], ((time_now - start_time) * 0.000000001))])), axis=0)
        start_time = time_now
    angles_mag = np.mean(angles_mag, axis=0)
    angles_gyr = np.mean(angles_gyr, axis=0)

    angles = (0.5 * angles_mag) + (0.5 * angles_gyr)
    return angles