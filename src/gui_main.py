#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import tkinter as tk
from tkinter import ttk
import queue
import threading
import time

class RoverGUI:
    def __init__(self, master):
        self.master = master
        master.title("Rover Control")

        # ROS initialization
        self.ros_initialized = False
        self.initialize_ros()

        # Queue for stats data
        self.queue = queue.Queue()

        # GUI setup
        self.setup_controls()
        self.setup_stats_display()
        self.setup_status_display()
        self.master.after(100, self.process_stats)
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)

    def initialize_ros(self):
        try:
            rospy.init_node('gui_node', anonymous=True)
            self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
            self.sub = rospy.Subscriber('/rover_stats', Float32MultiArray, self.stats_callback)
            self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
            self.ros_thread.start()
            self.ros_initialized = True
        except rospy.ROSException as e:
            print("ROS initialization failed:", str(e))
            self.master.after(1000, self.initialize_ros)

    def ros_spin(self):
        while not rospy.is_shutdown():
            time.sleep(0.1)

    def setup_controls(self):
        control_frame = ttk.Frame(self.master)
        control_frame.pack(padx=10, pady=10)

        # Front button
        front_btn = ttk.Button(control_frame, text="Front", 
                             command=lambda: self.publish_command(0.5, 0.0, "Moving forward"))
        front_btn.grid(row=0, column=1, padx=5, pady=5)

        # Left button
        left_btn = ttk.Button(control_frame, text="Left",
                            command=lambda: self.publish_command(0.0, 0.5, "Turning left"))
        left_btn.grid(row=1, column=0, padx=5, pady=5)

        # Stop button
        stop_btn = ttk.Button(control_frame, text="Stop",
                            command=lambda: self.publish_command(0.0, 0.0, "Stopped"))
        stop_btn.grid(row=1, column=1, padx=5, pady=5)

        # Right button
        right_btn = ttk.Button(control_frame, text="Right",
                             command=lambda: self.publish_command(0.0, -0.5, "Turning right"))
        right_btn.grid(row=1, column=2, padx=5, pady=5)

        # Back button
        back_btn = ttk.Button(control_frame, text="Back",
                            command=lambda: self.publish_command(-0.5, 0.0, "Moving backward"))
        back_btn.grid(row=2, column=1, padx=5, pady=5)

    def setup_stats_display(self):
        stats_frame = ttk.Frame(self.master)
        stats_frame.pack(padx=10, pady=10)

        self.battery_var = tk.StringVar(value="Battery: --%")
        self.speed_var = tk.StringVar(value="Speed: -- m/s")
        self.latency_var = tk.StringVar(value="Latency: -- ms")

        ttk.Label(stats_frame, textvariable=self.battery_var).grid(row=0, column=0, sticky="w")
        ttk.Label(stats_frame, textvariable=self.speed_var).grid(row=1, column=0, sticky="w")
        ttk.Label(stats_frame, textvariable=self.latency_var).grid(row=2, column=0, sticky="w")

    def setup_status_display(self):
        status_frame = ttk.Frame(self.master)
        status_frame.pack(padx=10, pady=10)
        self.status_var = tk.StringVar(value="Status: Ready")
        ttk.Label(status_frame, textvariable=self.status_var, 
                font=('Helvetica', 12, 'bold'), foreground='blue').pack()

    def stats_callback(self, msg):
        if len(msg.data) >= 3:
            battery = msg.data[0]
            speed = msg.data[1]
            latency = msg.data[2]
            self.queue.put((battery, speed, latency))

    def process_stats(self):
        try:
            while True:
                battery, speed, latency = self.queue.get_nowait()
                self.battery_var.set("Battery: %.1f%%" % battery)
                self.speed_var.set("Speed: %.2f m/s" % speed)
                self.latency_var.set("Latency: %.1f ms" % latency)
        except queue.Empty:
            pass
        self.master.after(100, self.process_stats)

    def publish_command(self, linear_x, angular_z, status):
        try:
            twist = Twist()
            twist.linear.x = linear_x
            twist.angular.z = angular_z
            self.pub.publish(twist)
            self.status_var.set("Status: " + status)
            print("Command sent:", status)  # Terminal confirmation
        except Exception as e:
            print("Error publishing command:", str(e))

    def on_closing(self):
        rospy.signal_shutdown("GUI closed")
        self.master.destroy()

if __name__ == '__main__':
    root = tk.Tk()
    app = RoverGUI(root)
    root.mainloop()

