#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header

class InspireHandDriver(Node):
    def __init__(self):
        super().__init__('inspire_hand_driver')

        # Serial connection parameters
        self.serial_port = "/dev/ttyUSB0"  # Update as needed
        self.baud_rate = 19200
        self.timeout = 1.0

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            self.get_logger().info(f"Connected to Inspire hand on {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Inspire hand: {e}")
            self.serial_conn = None

        # ROS 2 publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/inspire_hand/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Joint names for the Inspire hand (update as needed)
        self.joint_names = [
            # ...existing joint names...
        ]
        self.current_positions = [0.0] * len(self.joint_names)

        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.get_logger().info("Inspire Hand Driver initialized")
        self.test_connection()

    def test_connection(self):
        """Test the serial connection to the Inspire hand"""
        if not self.serial_conn:
            return
        try:
            test_cmd = "TEST\n"
            self.serial_conn.write(test_cmd.encode())
            self.get_logger().info(f"Sent test command: {test_cmd.strip()}")
            response = self.serial_conn.readline()
            if response:
                self.get_logger().info(f"Received response: {response.decode().strip()}")
            else:
                self.get_logger().info("No response received")
        except Exception as e:
            self.get_logger().error(f"Connection test failed: {e}")

    def send_joint_positions(self, positions):
        """Send joint positions to the Inspire hand"""
        if not self.serial_conn:
            return
        try:
            cmd = "MOVE " + " ".join([f"{pos:.3f}" for pos in positions]) + "\n"
            self.serial_conn.write(cmd.encode())
            self.get_logger().debug(f"Sent command: {cmd.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send joint positions: {e}")

    def trajectory_callback(self, msg):
        """Handle incoming trajectory commands"""
        self.get_logger().info(f"Received trajectory with {len(msg.points)} points")
        for point in msg.points:
            self.send_joint_positions(point.positions)
            self.current_positions = point.positions
            time.sleep(0.1)  # Basic timing, can be improved

    def publish_joint_states(self):
        """Publish current joint states"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_positions
        self.joint_state_pub.publish(joint_state)

    def destroy_node(self):
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    driver = InspireHandDriver()
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.get_logger().info("Shutting down Inspire Hand Driver")
    finally:
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
