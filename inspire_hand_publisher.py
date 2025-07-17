#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class InspireHandCommander(Node):
    def __init__(self):
        super().__init__('inspire_hand_commander')

        # Publisher to the driver
        self.publisher_ = self.create_publisher(JointTrajectory, '/inspire_hand/joint_trajectory', 10)

        # Send a trajectory after short delay to allow subscriber to connect
        self.timer = self.create_timer(5.0, self.send_command)

    def send_command(self):
        msg = JointTrajectory()
        
        # Optional: joint names (can be empty unless Inspire driver expects them)
        msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Create one point with 6 position values
        point = JointTrajectoryPoint()
        #point.positions = [100.0, 150.0, 200.0, 250.0, 100.0, 150.0]
        point.positions = [1.0, 0.0, 0.0, 1.0, 0.50, 1.0]
        point.time_from_start.sec = 1  # Optional
        
        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info('Sent joint trajectory command')
        self.timer.cancel()  # Only send once

def main(args=None):
    rclpy.init(args=args)
    commander = InspireHandCommander()
    rclpy.spin(commander)
    commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
