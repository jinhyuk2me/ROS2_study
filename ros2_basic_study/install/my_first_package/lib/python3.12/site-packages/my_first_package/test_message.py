#!/usr/bin/env python3

import rclpy as rp
from rclpy.node import Node
from my_first_package_msgs.msg import CmdAndPoseVel

class MessageTestNode(Node):
    def __init__(self):
        super().__init__('message_test_node')

        # Create a publisher
        self.publisher = self.create_publisher(CmdAndPoseVel, 'test_cmd_pose_vel', 10)

        # Create a timer to publish test messages
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Create a subscriber
        self.subscriber = self.create_subscription(
            CmdAndPoseVel,
            'test_cmd_pose_vel',
            self.subscriber_callback,
            10
        )

        self.get_logger().info('Message test node started')

    def timer_callback(self):
        msg = CmdAndPoseVel()
        msg.cmd_vel_linear = 1.0
        msg.cmd_vel_angular = 0.5
        msg.pose_x = 2.0
        msg.pose_y = 3.0
        msg.linear_vel = 1.2
        msg.angular_vel = 0.3

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: cmd_vel=({msg.cmd_vel_linear}, {msg.cmd_vel_angular}), '
                              f'pose=({msg.pose_x}, {msg.pose_y}), '
                              f'vel=({msg.linear_vel}, {msg.angular_vel})')

    def subscriber_callback(self, msg):
        self.get_logger().info(f'Received: cmd_vel=({msg.cmd_vel_linear}, {msg.cmd_vel_angular}), '
                              f'pose=({msg.pose_x}, {msg.pose_y}), '
                              f'vel=({msg.linear_vel}, {msg.angular_vel})')

def main(args=None):
    rp.init(args=args)
    node = MessageTestNode()
    rp.spin(node)
    node.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
