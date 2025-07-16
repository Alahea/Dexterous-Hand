#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class InspireHandCommander(Node):
    def __init__(self):
        super().__init__('inspire_hand_commander')

        # Check actual topic name used by driver
        self.publisher_ = self.create_publisher(JointTrajectory, '/inspire_hand/joint_trajectory', 10)

        # Wait a bit longer for subscribers
        self.timer = self.create_timer(2.0, self.send_command)

    def send_command(self):
        msg = JointTrajectory()

        # If unsure, try leaving joint_names empty
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        # msg.joint_names = []

        point = JointTrajectoryPoint()
        point.positions = [1.0, 1.5, 2.0, 2.5, 1.0, 1.5]  # Values in radians if expected
        point.time_from_start.sec = 1

        msg.points.append(point)
        self.publisher_.publish(msg)

        self.get_logger().info('Sent joint trajectory command')
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    commander = InspireHandCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
