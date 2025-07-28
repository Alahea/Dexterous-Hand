#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float32
import pandas as pd
import time

# Mapping from CSV joints to 6 DOFs
DOF_ANGLE_SOURCES = {
    0: ['LeftDigit5Metacarpophalangeal_flexion', 'LeftDigit5ProximalInterphalangeal_flexion'],
    1: ['LeftDigit4Metacarpophalangeal_flexion', 'LeftDigit4ProximalInterphalangeal_flexion'],
    2: ['LeftDigit3Metacarpophalangeal_flexion', 'LeftDigit3ProximalInterphalangeal_flexion'],
    3: ['LeftDigit2Metacarpophalangeal_flexion', 'LeftDigit2ProximalInterphalangeal_flexion'],
    4: ['LeftDigit1Metacarpophalangeal_flexion'],  # Thumb bend
    5: ['LeftDigit1Carpometacarpal_flexion'],      # Thumb rotate
}

ANGLE_LIMITS = {
    0: (19.0, 45.7),
    1: (19.0, 40.7),
    2: (19.0, 55.7),
    3: (19.0, 55.7),
    4: (-13.0, 53.6),   # originally (-13.0, 53.6) => (-33, 53.6) 
    5: (70.0, 80.0),    # originally (90.0, 100.0) => (40, 80)
}

def normalize_to_unit(angle_deg, min_deg, max_deg):
    angle_deg = max(min(angle_deg, max_deg), min_deg)
    return (angle_deg - min_deg) / (max_deg - min_deg)


class CSVToInspireHand(Node):
    def __init__(self, csv_path, frame_rate=100, default_force=400.0, default_speed=700.0):
        super().__init__('csv_to_inspire_hand')
        self.publisher = self.create_publisher(JointTrajectory, '/inspire_hand/joint_trajectory', 10)
        self.force_pub = self.create_publisher(Float32, '/inspire_hand/force', 10)
        self.speed_pub = self.create_publisher(Float32, '/inspire_hand/speed', 10)
        
        self.csv_data = pd.read_csv(csv_path)
        self.frame_delay = 1.0 / frame_rate  # seconds per frame
        self.default_force = default_force
        self.default_speed = default_speed

        if self.csv_data.empty:
            self.get_logger().error("CSV file is empty.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"Loaded CSV with {len(self.csv_data)} frames.")
        self.get_logger().info(f"Streaming at {frame_rate} FPS ({self.frame_delay:.3f}s per frame).")
        self.publish_trajectory()

    def extract_dof_positions(self, row):
        dof_positions = []
        for dof, joints in DOF_ANGLE_SOURCES.items():
            values = [row[j] for j in joints if j in row and pd.notnull(row[j])]
            if not values:
                dof_positions.append(0.0)
            else:
                avg = sum(values) / len(values)
                norm = normalize_to_unit(avg, *ANGLE_LIMITS[dof])
                dof_positions.append(norm)
        return dof_positions

    def publish_trajectory(self):
        for i, row in self.csv_data.iterrows():
            msg = JointTrajectory()
            msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

            point = JointTrajectoryPoint()
            point.positions = self.extract_dof_positions(row)
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(self.frame_delay * 1e9)

            msg.points.append(point)
            self.publisher.publish(msg)
            self.force_pub.publish(Float32(data=self.default_force))
            self.speed_pub.publish(Float32(data=self.default_speed))

            self.get_logger().info(f"Published frame {i+1}/{len(self.csv_data)}: {point.positions}")
            time.sleep(self.frame_delay)

def main(args=None):
    rclpy.init(args=args)
    node = CSVToInspireHand('Clip.csv', frame_rate=100)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
