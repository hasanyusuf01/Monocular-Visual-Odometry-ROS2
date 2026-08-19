import numpy as np
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# 1. Point this to the folder containing your recorded .db3 or .mcap file
bag_path = Path('imu_calibration_data')

# Lists to hold the extracted sensor data
accel_x, accel_y, accel_z = [], [], []
gyro_x, gyro_y, gyro_z = [], [], []

print(f"Opening ROS 2 bag: {bag_path}...")

# 2. Create a typestore for ROS 2 Humble to provide missing message definitions
typestore = get_typestore(Stores.ROS2_HUMBLE)

# 3. Open the bag and read the messages, passing the typestore
with AnyReader([bag_path], default_typestore=typestore) as reader:
    # Filter specifically for your IMU topic
    connections = [x for x in reader.connections if x.topic == '/mobile_sensor/imu']
    
    if not connections:
        print("Error: Could not find the topic '/mobile_sensor/imu' in the bag file.")
        exit()

    # Iterate through every message on that topic
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        # Deserialize the raw binary data into a Python object using the reader
        msg = reader.deserialize(rawdata, connection.msgtype)
        
        # Extract Linear Acceleration (m/s^2)
        accel_x.append(msg.linear_acceleration.x)
        accel_y.append(msg.linear_acceleration.y)
        accel_z.append(msg.linear_acceleration.z)
        
        # Extract Angular Velocity (rad/s)
        gyro_x.append(msg.angular_velocity.x)
        gyro_y.append(msg.angular_velocity.y)
        gyro_z.append(msg.angular_velocity.z)

print(f"Successfully extracted {len(accel_x)} IMU messages.")

# 4. Convert lists to NumPy arrays for mathematical analysis
accel_data = np.array([accel_x, accel_y, accel_z])
gyro_data = np.array([gyro_x, gyro_y, gyro_z])

# 5. Calculate Bias (Mean)
# A perfect accelerometer flat on a table should read [0.0, 0.0, 9.81]
# A perfect gyroscope should read [0.0, 0.0, 0.0]
accel_mean = np.mean(accel_data, axis=1)
gyro_mean = np.mean(gyro_data, axis=1)

# 6. Calculate Variance (Noise)
# This represents how much the sensor values jump around the mean
accel_var = np.var(accel_data, axis=1)
gyro_var = np.var(gyro_data, axis=1)

# 7. Print the results
print("\n--- IMU BIAS (Mean) ---")
print(f"Accel (X, Y, Z) [m/s^2]: {accel_mean}")
print(f"Gyro  (X, Y, Z) [rad/s]: {gyro_mean}")

print("\n--- IMU VARIANCE (Noise) ---")
print(f"Accel (X, Y, Z) : {accel_var}")
print(f"Gyro  (X, Y, Z) : {gyro_var}")
