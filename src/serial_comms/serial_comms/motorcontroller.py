#!/usr/bin/env python3

import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from serial_msgs.msg import MotorCurrents



class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher = self.create_publisher(
            msg_type=MotorControllerNode,
            topic='motor_currents',
            qos_profile = 1
        )
        self.timer = self.create_timer(0.02, self.send_velocity)



    def send_velocity(self):
        message = MotorCurrents()
        message.left_wheels = 100 
        message.right_wheels = 100
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)


    node = MotorControllerNode()


    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
