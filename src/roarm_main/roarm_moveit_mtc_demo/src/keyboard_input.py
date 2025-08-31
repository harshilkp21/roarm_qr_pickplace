#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time
import os

class SimulatedScannerNode(Node):
    def __init__(self):
        super().__init__('simulated_qr_scanner')

        self.publisher_ = self.create_publisher(String, 'qr_visitor', 10)
        self.publisher_pick_ = self.create_publisher(String, 'pick', 10)
        self.resume_subscriber = self.create_subscription(String, 'resume_scanner', self.resume_callback, 10)
        
        self.paused = False

        self.allowed_file = 'allowed_guests.txt'
        self.scanned_file = 'scanned_guests.txt'
        self.allowed_names = []
        self.scanned_names = set()
        self.cooldowns = {}
        self.name_index = 0

        self.load_allowed_names()
        self.clear_scanned_file()

        threading.Thread(target=self.listen_for_input, daemon=True).start()

    def load_allowed_names(self):
        if os.path.exists(self.allowed_file):
            with open(self.allowed_file, 'r') as f:
                self.allowed_names = [line.strip() for line in f.readlines()]
        else:
            self.get_logger().warn(f"{self.allowed_file} not found. No guests will be allowed.")

    def clear_scanned_file(self):
        with open(self.scanned_file, 'w') as f:
            f.write('')  # Clear file

    def save_name(self, name):
        with open(self.scanned_file, 'a') as f:
            f.write(f"{name}\n")

    def listen_for_input(self):
        while rclpy.ok():
            if self.paused:
                time.sleep(0.1)
                continue

            input("Press Enter or type 1 to simulate scan: ")

            if self.name_index >= len(self.allowed_names):
                self.get_logger().info("All allowed guests have been scanned.")
                continue

            name = self.allowed_names[self.name_index]
            self.name_index += 1

            current_time = time.time()
            last_time = self.cooldowns.get(name, 0)
            if current_time - last_time < 5.0:
                self.get_logger().info(f"Cooldown active for {name}, skipping.")
                continue

            self.cooldowns[name] = current_time
            self.publish_name(name)

            if name not in self.scanned_names:
                self.get_logger().info(f"Allowed guest {name}, first scan.")
                self.scanned_names.add(name)
                self.save_name(name)

                self.get_logger().info("Publishing pick message to robot.")
                self.publisher_pick_.publish(String(data="pick"))

                self.paused = True
                self.get_logger().info("Scanning paused until robot finishes.")
                threading.Thread(target=self.speak_welcome, args=(name,), daemon=True).start()
            else:
                self.get_logger().info(f"{name} already scanned. You already received your memento.")
                threading.Thread(target=self.speak_already_received, args=(name,), daemon=True).start()

    def publish_name(self, name):
        msg = String()
        msg.data = name
        self.publisher_.publish(msg)

    def resume_callback(self, msg):
        self.get_logger().info("Resuming scanner after robot completed pick.")
        self.paused = False

    def speak_welcome(self, name):
        self.get_logger().info(f"Welcome to TechXchange event, {name}.")

    def speak_already_received(self, name):
        self.get_logger().info(f"{name}, you have already received your memento.")

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedScannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.get_logger().info("Shutting down and clearing scanned_guests.txt.")
        node.clear_scanned_file()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
