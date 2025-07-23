#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pandas as pd

class InspireCSVMapper(Node):
    def __init__(self):
        super().__init__('inspire_csv_mapper')
        self.publisher_ = self.create_publisher(JointTrajectory, '/inspire_hand/joint_trajectory', 10)

        # Load CSV and map joints
        df = pd.read_csv('pinch_pick.csv')



        columns = ['LeftDigit5Metacarpophalangeal_flexion', 'LeftDigit4Metacarpophalangeal_flexion', 'LeftDigit5ProximalInterphalangeal_flexion', 'LeftDigit4ProximalInterphalangeal_flexion', 'LeftDigit5DistalInterphalangeal_flexion', 'LeftDigit4DistalInterphalangeal_flexion', 'LeftDigit3Metacarpophalangeal_flexion', 'LeftDigit3ProximalInterphalangeal_flexion', 'LeftDigit3DistalInterphalangeal_flexion', 'LeftDigit2Metacarpophalangeal_flexion', 'LeftDigit2ProximalInterphalangeal_flexion', 'LeftDigit2DistalInterphalangeal_flexion', 'LeftDigit1Carpometacarpal_flexion', 'LeftDigit1Metacarpophalangeal_flexion', 'LeftDigit1Interphalangeal_flexion']
        self.joint_names = ['pinky_proximal_joint', 'ring_proximal_joint', 'pinky_intermediate_joint', 'ring_intermediate_joint', 'pinky_tip_joint', 'ring_tip_joint', 'middle_proximal_joint', 'middle_intermediate_joint', 'middle_tip_joint', 'index_proximal_joint', 'index_intermediate_joint', 'index_tip_joint', 'thumb_proximal_yaw_joint', 'thumb_proximal_pitch_joint', 'thumb_distal_joint']

        data = df[columns].copy() * 0.01745  # deg to rad
        self.trajectory_data = data.values.tolist()

        self.gain = 0.95

        self.index = 0
        self.timer = self.create_timer(0.00910, self.send_command)

    def send_command(self):
        if self.index >= len(self.trajectory_data):
            return

        point = JointTrajectoryPoint()
        point.positions = [float(val * self.gain) for val in self.trajectory_data[self.index]]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 12000000

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [point]

        self.publisher_.publish(msg)
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = InspireCSVMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
