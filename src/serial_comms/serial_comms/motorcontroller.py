#!/usr/bin/env python3

import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from serial_msgs.msg import MotorCurrents, Feedback

STOP_VAL = 127
FORWARD_VAL = STOP_VAL + 27
BACKWARD_VAL = STOP_VAL - 27

class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.publisher = self.create_publisher(
            msg_type=MotorCurrents,
            topic='motor_currents',
            qos_profile = 1
        )
        # self.timer = self.create_timer(0.02, self.send_velocity)
        self.feedback_subscriber_ =  self.create_subscription(
            msg_type=Feedback,
            topic="feedback",
            callback=self.send_velocity,
            qos_profile=10
        )

    def send_velocity(self, feedback):
        message = MotorCurrents()
        if feedback.front < 60:
            if feedback.left < 30 and feedback.right > 30: # turn right
                message.left_wheels = FORWARD_VAL
                message.right_wheels = BACKWARD_VAL
            elif feedback.left < 30 and feedback.right > 30: # turn left
                message.left_wheels = BACKWARD_VAL
                message.right_wheels = FORWARD_VAL
            else: # stop
                message.left_wheels = STOP_VAL
                message.right_wheels = STOP_VAL
        else:
            message.left_wheels = FORWARD_VAL
            message.right_wheels = FORWARD_VAL
        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)


    node = MotorControllerNode()


    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
