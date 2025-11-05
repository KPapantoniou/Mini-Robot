import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

data = pd.read_csv("D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\results_robot_single.csv")
data["Theta_left_deg"] = np.degrees(data["Theta_left"])

save_dir = r"D:\panepistimio\Thesis\Micro-robot\thesis\photos"
os.makedirs(save_dir, exist_ok=True)

# 1. Position, Velocity, Acceleration over Angle
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(data["Theta_left_deg"], data["Pos_x"], label="Position Horizontal (m)", color='blue')
plt.xlabel("Angle (deg)")
plt.ylabel("Position (m)")
plt.title("Position Horizontal Over Angle")
plt.grid()

plt.subplot(3, 1, 2)
plt.plot(data["Theta_left_deg"], data["Vel_x"], label="Velocity X (m/s)", color='red')
plt.xlabel("Angle (deg)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity X Over Angle")
plt.grid()

plt.subplot(3, 1, 3)
plt.plot(data["Theta_left_deg"], data["Acc_x"], label="Acceleration X (m/s^2)", color='green')
plt.xlabel("Angle (deg)")
plt.ylabel("Acceleration (m/s^2)")
plt.title("Acceleration Over Angle")
plt.grid()

plt.tight_layout()
# plt.savefig(os.path.join(save_dir, "position_velocity_acceleration.svg"))
plt.show()

# 2. Forces Over Time
plt.figure(figsize=(10, 6))
plt.subplot(2, 1, 1)
plt.plot(data["Time"], data["Force_x"], label="Force Y (N)", color='orange')
plt.plot(data["Time"], data["Total_Force_Z"], label="Force Z (N)", color='blue')
plt.xlabel("Time (ms)")
plt.ylabel("Force (N)")
plt.title("Forces Over Time")
plt.legend()
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(data["Time"], data["Coulomb_Force"], label="Coulomb Force (N)", color='green')
plt.plot(data["Time"], data["Friction_x"], label="Friction Force (N)", color='gray')
plt.xlabel("Time (ms)")
plt.ylabel("Force (N)")
plt.title("Friction Forces Over Time")
plt.legend()
plt.grid()

plt.tight_layout()
# plt.savefig(os.path.join(save_dir, "forces_over_time.svg"))
plt.show()