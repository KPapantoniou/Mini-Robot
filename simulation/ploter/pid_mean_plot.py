import pandas as pd
import matplotlib.pyplot as plt

df1 = pd.read_csv("D:\\panepistimio\\Thesis\\Micro-robot\\code\\simulation\\sim_pid_mean.csv")

plt.figure(figsize=(12, 6))
plt.plot(df1["Time"], df1["TargetSpeed"], label="PID Motor Speed", color="blue")
plt.plot(df1["Time"], df1["Speed_left"], label="Left Motor Speed", color="orange", linestyle="--")
plt.plot(df1["Time"], df1["Speed_right"], label="Right Motor Speed", color="yellow",linestyle="--")
plt.plot(df1["Time"], df1["Mean_speed"], label="Mean Speed", color="green")
# plt.plot(df1["Time"], df1["TargetSpeed"], label="Target Speed", linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("Speed (rad/s)")
plt.title("PID Motor Response")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()