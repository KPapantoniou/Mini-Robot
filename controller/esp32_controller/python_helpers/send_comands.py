import socket
import keyboard 
import time
import threading

ESP32_IP = "192.168.180.188" 
ESP32_PORT = 1234           
DEBUG_PORT = 9999

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
debug_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
debug_sock.bind(("0.0.0.0", DEBUG_PORT))

KEY_BINDINGS = {
    'w': 'F',  
    's': 'B',  
    'a': 'L',  
    'd': 'R',  
    'space': 'S',  
    '+': '+',  
    '-': '-',  
}

def receive_debug_logs():
    print(f"Listening for ESP32 debug messages on port {DEBUG_PORT}...\n")
    while True:
        data, addr = debug_sock.recvfrom(1024) 
        print(f"[ESP32] {data.decode()}")  


debug_thread = threading.Thread(target=receive_debug_logs, daemon=True)
debug_thread.start()

print("Press W/A/S/D to move, SPACE to stop. Press q to exit.")

try:
    while True:
        for key, command in KEY_BINDINGS.items():
            if keyboard.is_pressed(key):
                sock.sendto(command.encode(), (ESP32_IP, ESP32_PORT))
                print(f"Sent: {command}")
                time.sleep(0.05)  
        if keyboard.is_pressed('q'):
            break

        time.sleep(0.01)  

except KeyboardInterrupt:
    print("\nExiting program.")

finally:
    sock.close()
    debug_sock.close()