import pandas as pd
import matplotlib.pyplot as plt
import os

data = pd.read_csv(r"D:\panepistimio\Thesis\Micro-robot\code\simulation\results_motor.csv")

save_dir = r"D:\panepistimio\Thesis\Micro-robot\thesis\photos"
os.makedirs(save_dir, exist_ok=True)

# 1. Angular position over time
plt.figure(figsize=(6, 3))
plt.plot(data["Time"], data["Theta_right"], label="Theta Right (Angular Position in rad)", color='blue')
plt.xlabel("Time (ms)")
plt.ylabel("Theta (rad)")
plt.title("Angular Position Over Time")
plt.grid()
plt.tight_layout()
plt.savefig(os.path.join(save_dir, "angular_position.svg"))
plt.show()

# 2. Angular velocity over time
plt.figure(figsize=(6, 3))
plt.plot(data["Time"], data["Omega_right"], label="Omega Right (Angular Velocity)", color='orange')
plt.xlabel("Time (s)")
plt.ylabel("Omega (rad/sec)")
plt.title("Angular Velocity Over Time")
plt.grid()
plt.tight_layout()
plt.savefig(os.path.join(save_dir, "angular_velocity.svg"))
plt.show()

# 3. Y-Axis Centrifugal Force Over Time
plt.figure(figsize=(6, 3))
plt.plot(data["Time"], data["Force_y_right"], label="fy Motor (N)", color='green')
plt.xlabel("Time (s)")
plt.ylabel("fy (N)")
plt.title("Y-Axis Centrifugal Force Over Time")
plt.grid()
plt.tight_layout()
# plt.savefig(os.path.join(save_dir, "force_y.svg"))
plt.show()

# 4. Z-Axis Centrifugal Force Over Time
plt.figure(figsize=(6, 3))
plt.plot(data["Time"], data["Force_z_right"], label="fz Right Motor (N)", color='grey')
plt.xlabel("Time (s)")
plt.ylabel("fz (N)")
plt.title("Z-Axis Centrifugal Force Over Time")
plt.grid()
plt.tight_layout()
# plt.savefig(os.path.join(save_dir, "force_z.svg"))
plt.show()