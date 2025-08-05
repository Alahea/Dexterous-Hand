#!/usr/bin/env python3
#sample code to do a certain pose, adjasent to the EMG_to_inspire


import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class EMGToInspire(Node):
    def __init__(self):
        super().__init__('emg_to_inspire')

        self.sub = self.create_subscription(Int32, '/emg_gesture', self.callback, 10)
        self.pub = self.create_publisher(JointTrajectory, '/inspire_hand/joint_trajectory', 10)

        # Gesture to pose mapping
        self.poses = {
            1: [0.8, 0.8, 0.8, 0.8, 0.5, 0.7],  # Grip
            2: [0.0, 0.0, 0.0, 1.0, 0.5, 0.5],  # Pinch
            3: [0.0, 0.0, 0.0, 1.0, 0.0, 0.5],  # Point
            4: [0.0, 0.0, 0.0, 0.0, 1.0, 1.0],  # Thumbs up
        }

        # Stability filter
        self.last_gesture = 0
        self.candidate_gesture = None
        self.candidate_count = 0
        self.required_stability = 3  # times in a row before accepting

        self.get_logger().info("EMG to Inspire hand node started with zero as bridge.")

    def send_pose(self, gesture):
        traj = JointTrajectory()
        traj.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        point = JointTrajectoryPoint()
        point.positions = self.poses[gesture]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500000000  # 0.5s

        traj.points.append(point)
        self.pub.publish(traj)
        self.get_logger().info(f"Sent hand pose for gesture {gesture}")

    def callback(self, msg):
        gesture = msg.data

        # Ignore zero completely (no movement)
        if gesture == 0:
            self.get_logger().info("Zero detected → bridge mode (no movement)")
            self.candidate_gesture = None
            self.candidate_count = 0
            return

        # Handle new candidate
        if gesture != self.candidate_gesture:
            self.candidate_gesture = gesture
            self.candidate_count = 1
            self.get_logger().info(f"New candidate gesture {gesture}, count reset to 1")
        else:
            self.candidate_count += 1
            self.get_logger().info(f"Candidate gesture {gesture} count = {self.candidate_count}")

        # If stable enough and different from last executed pose
        if (
            self.candidate_count >= self.required_stability
            and gesture != self.last_gesture
        ):
            self.send_pose(gesture)
            self.last_gesture = gesture
            self.candidate_count = 0  # reset after accepting

def main(args=None):
    rclpy.init(args=args)
    node = EMGToInspire()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
