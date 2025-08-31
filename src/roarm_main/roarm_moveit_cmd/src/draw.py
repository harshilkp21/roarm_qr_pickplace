#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from roarm_msgs.srv import MoveJointCmd
import time
 
class GcodeParserNode(Node):
 
    def __init__(self):
        super().__init__('gcode_parser')
        self.cli = self.create_client(MoveJointCmd, '/move_joint_cmd')
 
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /move_joint_cmd service...')
 
        self.roll = -2.343765986151993e-05
        self.pitch = -2.9150777356878876e-05
        self.yaw = 0.0
 
        self.current_pos = [0.0, 0.0, 5.0]  # X, Y, Z
        self.pen_down = False
 
        self.prev_x = self.current_pos[0]
        self.prev_y = self.current_pos[1]
 
        gcode_file = '/home/harshil/Music/gcode/full_detail_drawing.gcode'  # Adjust path
        self.parse_gcode(gcode_file)
 
    def send_cmd(self, x, y, z):
        req = MoveJointCmd.Request()
        req.x = x
        req.y = y
        req.z = z
        req.roll = self.roll
        req.pitch = self.pitch
        req.yaw = self.yaw
        self.cli.call_async(req)
 
    def parse_gcode(self, filename):
        with open(filename, 'r') as f:
            lines = f.readlines()
 
        for line in lines:
            line = line.strip()
 
            if not line or line.startswith(';'):
                continue  # Skip comments or empty lines
 
            parts = line.split()
 
            if parts[0] in ['G21', 'G90']:
                continue  # Skip unit and positioning commands
 
            if parts[0] in ['G0', 'G1']:
                x = y = z = None
 
                for part in parts[1:]:
                    if part.startswith('X'):
                        x = float(part[1:])
                    elif part.startswith('Y'):
                        y = float(part[1:])
                    elif part.startswith('Z'):
                        z = float(part[1:])
 
                if z is not None:
                    self.current_pos[2] = z
                    self.pen_down = self.current_pos[2] <= 0
 
                if x is not None:
                    self.current_pos[0] = x
                if y is not None:
                    self.current_pos[1] = y
 
                # Send movement only if pen is down AND it's a G1 move AND X or Y changed
                if self.pen_down and parts[0] == 'G1' and (x is not None or y is not None):
                    self.get_logger().info(
                        f"Drawing line: ({self.prev_x}, {self.prev_y}) -> ({self.current_pos[0]}, {self.current_pos[1]})"
                    )
                    self.send_cmd(self.current_pos[0], self.current_pos[1], self.current_pos[2])
                    time.sleep(0.2)  # Tune for speed
                else:
                    # For non-drawing moves, just update position
                    self.send_cmd(self.current_pos[0], self.current_pos[1], self.current_pos[2])
                    time.sleep(1)
 
                self.prev_x = self.current_pos[0]
                self.prev_y = self.current_pos[1]
 
        self.get_logger().info('G-code execution complete.')
 
def main():
    rclpy.init()
    node = GcodeParserNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
 