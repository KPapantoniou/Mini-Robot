import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

data = pd.read_csv("C:\\Users\\HP\\Desktop\\Micro-robot-main\\code\\controller\\robot_controller\\camera_controller\\build\\Debug\\robot_path.csv")

plt.figure(figsize=(15,5))
plt.subplot(1,3,1)
plt.plot(data["time"],data["front_y"])
plt.plot(data["time"].iloc[-1],data["target_y"].iloc[-1],'go',label="Target" )
plt.xlabel("Time (s)")
plt.ylabel("Position (cm)")
plt.title("X Position Over Time")
plt.grid()

plt.subplot(1,3,2)
plt.plot(data["time"],data["front_x"])
plt.plot(data["time"].iloc[-1],data["target_x"].iloc[-1],'go',label="Target")
plt.xlabel("Time (s)")
plt.ylabel("Position (cm)")
plt.title("Y Position Over Time")   
plt.grid()

plt.subplot(1,3,3)
plt.plot(data["front_y"], data["front_x"])
plt.plot(data["target_y"].iloc[-1],data["target_x"].iloc[-1],'go',label="Target")
plt.xlabel("X Position (cm)")
plt.ylabel("Y Position (cm)")
plt.title("Y Position Over X")

# plt.axis('equal')

# # 🔧 Force equal aspect ratio
# ax = plt.gca()
# ax.set_aspect('equal')

# # 🔧 Set consistent tick spacing (optional)
# ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
# ax.yaxis.set_major_locator(ticker.MultipleLocator(5))
# plt.ylim(data["target_x"].iloc[0],data["target_x"].iloc[-1])
# plt.xlim(data["target_y"].iloc[0],data["target_y"].iloc[-1])

plt.grid()
plt.tight_layout()
plt.show()

plt.figure(figsize=(15,5))
plt.plot(data["time"],data["debug_distance"])
plt.xlabel("Time")
plt.ylabel("Distance pixel")
plt.grid()
plt.tight_layout()
plt.show()