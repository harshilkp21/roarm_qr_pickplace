#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(3.0, self.timer_callback)
        self.step = 0

        # Joint names
        self.joint_names = [
            'base_link_to_link1',
            'link1_to_link2',
            'link2_to_link3',
            'link3_to_link4',
            'link4_to_link5',
            'link5_to_gripper_link'
        ]

        # Arm positions
        self.pick_position = [-0.764, 0.484, 2.117, 0.0, 0.0]
        self.place_position = [0.187, 0.484, 1.754, 0.0, 0.0]

        # Gripper values
        self.gripper_open = 1.305
        self.gripper_closed = 0.0

        self.get_logger().info("Pick-and-place node started!")

    def publish_joint_state(self, arm_position, gripper_position):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = arm_position + [gripper_position]
        self.publisher_.publish(msg)

    def timer_callback(self):
        if self.step == 0:
            self.get_logger().info("Step 0: Open gripper")
            self.publish_joint_state(self.pick_position, self.gripper_open)
            self.step += 1

        elif self.step == 1:
            self.get_logger().info("Step 1: Move to PICK position")
            self.publish_joint_state(self.pick_position, self.gripper_open)
            self.step += 1

        elif self.step == 2:
            self.get_logger().info("Step 2: Close gripper to pick")
            self.publish_joint_state(self.pick_position, self.gripper_closed)
            self.step += 1

        elif self.step == 3:
            self.get_logger().info("Step 3: Move to PLACE position")
            self.publish_joint_state(self.place_position, self.gripper_closed)
            self.step += 1

        elif self.step == 4:
            self.get_logger().info("Step 4: Open gripper to release")
            self.publish_joint_state(self.place_position, self.gripper_open)
            self.step += 1

        else:
            self.get_logger().info("✅ Pick and place complete. Node stopping.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
