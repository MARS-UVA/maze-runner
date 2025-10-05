#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from serial_msgs.msg import MotorCurrents, Feedback
from geometry_msgs.msg import Twist

class SampleNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.feedback_queue = []
        self.motor_publisher_ = self.create_publisher(
            msg_type=MotorCurrents,
            topic='motor_currents',
            qos_profile=1
        )
        self.feedback_subscriber_ = self.create_subscription(
            msg_type=Feedback,
            topic='feedback',
            qos_profile=1,
            callback=self.send_motor_currents
        )
    
    def send_motor_currents(self, feedback):
        self.get_logger().info("Publishing motor currents")
        message = MotorCurrents()

        message.left_wheels = 154
        message.right_wheels = 154

        if feedback.front_sensor < 30:
            message.left_wheels = 127
            message.right_wheels = 127
        elif feedback.right_sensor < 30:
            message.left_wheels = 127
            message.right_wheels = 127
        elif feedback.left_sensor < 30:
            message.left_wheels = 127
            message.right_wheels = 127

        self.motor_publisher_.publish(message)

    
def main(args=None):
    rclpy.init(args=args)

    node = SampleNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()