import matplotlib.pyplot as plt 
import csv
def plot_data(filename = "C:\\Users\\HP\\Desktop\\Micro-robot-main\\code\\controller\\wifi_controller\\motor_speeds.csv"):
    times = []
    speeds_a = []
    speeds_b = []

    with open(filename, mode = 'r')as file:
        reader = csv.reader(file)
        next(reader)
        for row in reader:
            times.append(float(row[0]))
            speeds_a.append(float(row[1]))
            speeds_b.append(float(row[2]))

    plt.figure()
    plt.plot(times,speeds_a, label='Motor A', color='blue')
    plt.plot(times, speeds_b, label='Motor B', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('rad/s')
    plt.title('Logged Motor Speeds')
    plt.legend()
    plt.grid(True)
    plt.show()

plot_data()