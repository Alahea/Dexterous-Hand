#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from inspire_hand_interfaces.srv import SetPos, SetSpeed, SetForce, GetPosAct  # Update with your actual ROS 2 package and service names

class InspireHandClient(Node):
    def __init__(self):
        super().__init__('inspire_hand_python_client')
        self.set_force_cli = self.create_client(SetForce, 'inspire_hand/set_force')
        self.set_speed_cli = self.create_client(SetSpeed, 'inspire_hand/set_speed')
        self.set_pos_cli = self.create_client(SetPos, 'inspire_hand/set_pos')
        self.get_pos_cli = self.create_client(GetPosAct, 'inspire_hand/get_pos_act')

        self.get_logger().info("Waiting for Inspire Hand services...")
        for cli in [self.set_force_cli, self.set_speed_cli, self.set_pos_cli, self.get_pos_cli]:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for service {cli.srv_name}...')
        self.get_logger().info("All services are available.")

    def call_set_force(self, force):
        req = SetForce.Request()
        req.force0 = req.force1 = req.force2 = req.force3 = req.force4 = force
        future = self.set_force_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Force set successfully.")

    def call_set_speed(self, speed):
        req = SetSpeed.Request()
        req.speed0 = req.speed1 = req.speed2 = req.speed3 = req.speed4 = speed
        future = self.set_speed_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Speed set successfully.")

    def call_set_position(self, pos):
        req = SetPos.Request()
        req.pos0 = req.pos1 = req.pos2 = req.pos3 = req.pos4 = req.pos5 = pos
        future = self.set_pos_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Position command sent.")

    def get_current_position(self):
        req = GetPosAct.Request()
        future = self.get_pos_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        self.get_logger().info(f"Current positions: {resp.curpos}")

def main(args=None):
    rclpy.init(args=args)
    client = InspireHandClient()

    client.get_logger().info("Starting hand control sequence...")

    client.call_set_force(500)
    client.call_set_speed(300)

    client.get_logger().info("Opening hand...")
    client.call_set_position(200)
    rclpy.sleep(3)

    client.get_logger().info("Closing hand...")
    client.call_set_position(1800)
    rclpy.sleep(3)

    client.get_logger().info("Getting current hand positions...")
    client.get_current_position()

    client.get_logger().info("Sequence complete.")
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()#main cide
