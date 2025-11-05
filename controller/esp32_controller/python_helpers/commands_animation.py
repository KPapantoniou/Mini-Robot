import socket
import keyboard
import time
import threading
import re
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import csv
import os
from datetime import datetime


ESP32_IP = "192.168.234.188"
ESP32_PORT = 1234
DEBUG_PORT = 9999
 
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
debug_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
debug_sock.bind(("0.0.0.0", DEBUG_PORT))

max_points = 200
times = deque(maxlen=max_points)
speeds_A = deque(maxlen=max_points)
speeds_B = deque(maxlen=max_points)

pattern = re.compile(r"Motor (A|B) - Voltage \(V\): [\d\.]+, RPM: ([\d\.]+)")

csv_filename = f"C:\\Users\\HP\\Desktop\\Micro-robot-main\\code\\controller\\wifi_controller\\motor_speeds.csv"
csv_file = open(csv_filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time (s)', 'Motor A RPM', 'Motor B RPM'])

def receive_debug_logs():
    while True:
        try:
            data, _ = debug_sock.recvfrom(1024)
            message = data.decode().strip()
            print(f"[ESP32] {data.decode()}")
            current_time = time.time() - start_time

            
            matches = pattern.findall(message)
            # if matches:
            times.append(current_time)
            rpm_a = None
            rpm_b = None
            for motor, rpm in matches:
                rpm = float(rpm)
                if motor == 'A':
                    speeds_A.append(rpm)
                    rpm_a = rpm
                elif motor == 'B':
                    speeds_B.append(rpm)
                    rpm_b = rpm
                    
            last_a = speeds_A[-1] if speeds_A else 0
            last_b = speeds_B[-1] if speeds_B else 0
            csv_writer.writerow([current_time, rpm_a if rpm_a is not None else last_a, rpm_b if rpm_b is not None else last_b])
       
        except Exception as e:
            print(f"Error: {e}")

def measure_udp_latency(repeats=10):
    total_time = 0
    for i in range(repeats):
        try:
            start = time.time()
            sock.sendto(b"PING", (ESP32_IP, ESP32_PORT))
            sock.settimeout(2.0)  # Increase timeout to 2 seconds
            response, _ = sock.recvfrom(1024)
            end = time.time()
            if response.strip() == b"PONG":
                rtt = (end - start) * 1000  # ms
                total_time += rtt
                print(f"UDP RTT {i+1}: {rtt:.2f} ms")
        except socket.timeout:
            print("Timeout: No response from ESP32")
    avg = total_time / repeats
    print(f"Average UDP RTT: {avg:.2f} ms")



fig, ax = plt.subplots()
line_a, = ax.plot([], [], label='Motor A', color='blue')
line_b, = ax.plot([], [], label='Motor B', color='red')
ax.set_xlabel('Time (s)')
ax.set_ylabel('RPM')
ax.set_title('Real-time Motor Speeds')
ax.legend()
ax.grid(True)

def init():
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 1200)
    return line_a, line_b

def update(frame):
    if len(times) == 0:
        return line_a, line_b

    min_len = min(len(times), len(speeds_A), len(speeds_B))
    x = list(times)[-min_len:]
    y_a = list(speeds_A)[-min_len:]
    y_b = list(speeds_B)[-min_len:]

    current_time = time.time() - start_time
    ax.set_xlim(max(0, current_time - 10), current_time)

    line_a.set_data(x, y_a)
    line_b.set_data(x, y_b)

    # Προστασία από κενές λίστες
    if y_a or y_b:
        max_rpm = max(max(y_a, default=0), max(y_b, default=0)) * 1.1
        ax.set_ylim(0, max(100, max_rpm))  # Πάντα >= 100
    else:
        ax.set_ylim(0, 100)

    return line_a, line_b




start_time = time.time()
debug_thread = threading.Thread(target=receive_debug_logs, daemon=True)
debug_thread.start()


ani = animation.FuncAnimation(
    fig, 
    update, 
    init_func=init,
    interval=50, 
    blit=True
)

def command_loop():
    KEY_BINDINGS = {
        'w': 'F', 'b': 'B',
        'a': 'L', 'd': 'R',
        'space': 'S',
        '+': '+', '-': '-',
        'q': 'Q','l':'LAT'
    }
    
    print("Controls: W/S/A/D - Movement, +/- - Speed, SPACE - Stop")
    
    while True:
        try:
            for key, cmd in KEY_BINDINGS.items():
                if keyboard.is_pressed(key):
                    if key == 'q': 
                        # plt.close('all')
                        break
                    elif key =='l':
                        measure_udp_latency()
                    else:
                        sock.sendto(cmd.encode(), (ESP32_IP, ESP32_PORT))
                    time.sleep(0.1)
            time.sleep(0.01)
        except Exception as e:
            print(f"Error: {e}")
            break

command_thread = threading.Thread(target=command_loop, daemon=True)
command_thread.start()
try:
    while plt.fignum_exists(fig.number):
        plt.pause(0.01)
except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    sock.close()
    debug_sock.close()
    csv_file.close()