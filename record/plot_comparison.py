import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Path to the CSV file
csv_file1 = "/home/athena/airo_observer_ws/src/airo_control_interface/airo_control/rosbag_logs/rosbag_debug_ekf_init.csv"
csv_file2 = "/home/athena/airo_observer_ws/src/airo_control_interface/airo_control/rosbag_logs/rosbag_debug_rd3_init.csv"

# Read the CSV file into a pandas DataFrame
df1 = pd.read_csv(csv_file1, names=["Time", "local_x", "local_y", "local_z", "accel_disturbance_x", "accel_disturbance_y", "accel_disturbance_z"])
df2 = pd.read_csv(csv_file2, names=["Time", "local_x", "local_y", "local_z", "accel_disturbance_x", "accel_disturbance_y", "accel_disturbance_z"])

# Extract the data columns
time1 = df1["Time"].to_numpy()
x1 = df1["local_x"].to_numpy()
y1 = df1["local_y"].to_numpy()
z1 = df1["local_z"].to_numpy()
dis_x1 = df1["accel_disturbance_x"].to_numpy()
dis_y1 = df1["accel_disturbance_y"].to_numpy()
dis_z1 = df1["accel_disturbance_z"].to_numpy()

time2 = df2["Time"].to_numpy()
x2 = df2["local_x"].to_numpy()
y2 = df2["local_y"].to_numpy()
z2 = df2["local_z"].to_numpy()
dis_x2 = df2["accel_disturbance_x"].to_numpy()
dis_y2 = df2["accel_disturbance_y"].to_numpy()
dis_z2 = df2["accel_disturbance_z"].to_numpy()

# Set the initial time as the first data point of the time column
init_time1 = time1[0]
init_time2 = time2[0]

# Subtract the initial time from the time column
time1 = time1 - init_time1
time2 = time2 - init_time2

# Create a figure with subplots for each line graph
fig, axs = plt.subplots(6, 1, figsize=(8, 12))

# Plot the line graphs
axs[0].plot(time1, x1, label="ekf")
axs[0].plot(time2, x2, label="rd3")
axs[0].set_ylabel("local_x")

axs[1].plot(time1, y1, label="ekf")
axs[1].plot(time2, y2, label="rd3")
axs[1].set_ylabel("local_y")

axs[2].plot(time1, z1, label="ekf")
axs[2].plot(time2, z2, label="rd3")
axs[2].set_ylabel("local_z")

axs[3].plot(time1, dis_x1, label="ekf")
axs[3].plot(time2, dis_x2, label="rd3")
axs[3].set_ylabel("accel_disturbance_x")

axs[4].plot(time1, dis_y1, label="ekf")
axs[4].plot(time2, dis_y2, label="rd3")
axs[4].set_ylabel("accel_disturbance_y")

axs[5].plot(time1, dis_z1, label="ekf")
axs[5].plot(time2, dis_z2, label="rd3")
axs[5].set_ylabel("accel_disturbance_z")

# Set common x-axis label
axs[-1].set_xlabel("Time")

# Add legend
axs[0].legend()
axs[1].legend()
axs[2].legend()
axs[3].legend()
axs[4].legend()
axs[5].legend()

# Adjust spacing between subplots
fig.tight_layout()

# Display the graphs
plt.show()
