#!/usr/bin/env python3

import tkinter as tk
import subprocess
import yaml
import os


yaml_file_path = os.path.expanduser('~/robot_2030a_ws/src/robot_2030a_controller/config/speed_params.yaml')

def arm_launch():
    try:
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'source ~/robot_2030a_ws/install/setup.bash && ros2 launch robot_2030a_bringup real_robot.launch.py'])
    except Exception as e:
        print(f"Error launching arm: {e}")

def start_socket_tcp_connection():
    try:
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', 'source ~/robot_2030a_ws/install/setup.bash && python3 ~/robot_2030a_ws/src/robot_2030a_interface/ros_socket_tcp.py'])
    except Exception as e:
        print(f"Error starting socket connection: {e}")

def close_all_terminals():
    try:
        subprocess.Popen(['pkill', '-f', 'gnome-terminal']) 
    except Exception as e:
        print(f"Error closing terminals: {e}")

def send_command(entry):
    try:
        command = entry.get()

       
        with open(yaml_file_path, 'r') as file:
            yaml_data = yaml.safe_load(file)

        
        yaml_data['speed'] = int(command)  

        
        with open(yaml_file_path, 'w') as file:
            yaml.dump(yaml_data, file)

    except Exception as e:
        print(f"Error updating YAML file: {e}")

def main():
    root = tk.Tk()
    root.title("ROS 6dof Arm Launch Starter")

    root.geometry("400x200")

    label = tk.Label(root, text="Click the button to start the 6dof robotic arm:")
    label.pack(pady=10)

    start_custom_launch_button = tk.Button(root, text="Start ROS 6dof Arm", command=arm_launch)
    start_custom_launch_button.pack()

    start_micro_controller_button = tk.Button(root, text="Connect Robot", command=start_socket_tcp_connection)
    start_micro_controller_button.pack()

    close_terminals_button = tk.Button(root, text="Close All Terminals", command=close_all_terminals)
    close_terminals_button.pack()

    input_frame = tk.Frame(root)
    input_frame.pack(pady=10)

    input_label = tk.Label(input_frame, text="Enter speed:")
    input_label.grid(row=0, column=0)

    input_entry = tk.Entry(input_frame, width=10)
    input_entry.grid(row=0, column=1)

    send_button = tk.Button(root, text="Send", command=lambda: send_command(input_entry))
    send_button.pack(pady=10)
    send_button.place(in_=input_frame, relx=1.4, rely=0.0, anchor=tk.NE)

    root.mainloop()

if __name__ == "__main__":
    main()
