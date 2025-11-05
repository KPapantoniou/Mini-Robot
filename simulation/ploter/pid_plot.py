import pandas as pd
import matplotlib.pyplot as plt

# Load CSV data

df1 = pd.read_csv("D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\sim_pid.csv")
df2 = pd.read_csv("D:\\panepistimio\\Thesis\Micro-robot\\code\\simulation\\sim_pd.csv")


# Plot speed and target

plt.figure(figsize=(12, 6))
plt.plot(df1["Time"], df1["Speed"], label="PID Motor Speed", color="blue")
plt.plot(df2["Time"], df2["Speed"], label="PD Motor Speed", color="orange")
# plt.plot(df1["Time"], df1["TargetSpeed"], label="Target Speed", linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("Speed (rad/s)")
plt.title("PID Motor Response")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Optional: Plot error and voltage

plt.figure(figsize=(12, 4))
plt.plot(df1["Time"], df1["Error"], label="PID Error", color="red")
plt.plot(df2["Time"], df2["Error"], label="PD Error", color="green")
plt.plot(df1["Time"], df1["Voltage"], label="Voltage", linestyle="--")
plt.xlabel("Time (s)")
plt.title("Error and Control Voltage")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
