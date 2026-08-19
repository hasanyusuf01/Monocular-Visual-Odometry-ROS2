import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
import allantools

bag_path = Path('imu_calibration_data')

# Lists for data and timestamps
gyro_z = []
timestamps = []

print(f"Opening ROS 2 bag: {bag_path}...")
typestore = get_typestore(Stores.ROS2_HUMBLE)

with AnyReader([bag_path], default_typestore=typestore) as reader:
    connections = [x for x in reader.connections if x.topic == '/mobile_sensor/imu']
    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        
        # Extracting Gyro Z-axis for this example
        gyro_z.append(msg.angular_velocity.z)
        timestamps.append(timestamp)

gyro_data = np.array(gyro_z)
timestamps = np.array(timestamps)

# Calculate exact sample rate in Hz from ROS timestamps
dt_seconds = np.diff(timestamps) / 1e9
rate = 1.0 / np.mean(dt_seconds)
print(f"Calculated IMU Sample Rate: {rate:.2f} Hz")

# Define averaging times (taus) from shortest to 1/3 of the total dataset length
max_tau = (len(timestamps) / rate) / 3
taus = np.logspace(np.log10(1.0/rate), np.log10(max_tau), 50)

# Calculate Overlapping Allan Deviation for the frequency (rate) data
t_gyro, ad_gyro, ade, adn = allantools.oadev(gyro_data, rate=rate, data_type="freq", taus=taus)

# Plotting on a log-log scale
plt.figure(figsize=(10, 6))
plt.loglog(t_gyro, ad_gyro, label='Gyro Z-Axis', color='blue', linewidth=2)

plt.grid(True, which="both", ls="-", color='0.8')
plt.xlabel(r"Averaging Time $\tau$ (s)")
plt.ylabel(r"Allan Deviation $\sigma(\tau)$")
plt.title("IMU Allan Deviation Analysis")
plt.legend()
plt.show()
