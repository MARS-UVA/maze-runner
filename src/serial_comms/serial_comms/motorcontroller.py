#!/usr/bin/env python3

import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from serial_msgs.msg import MotorCurrents, Feedback

STOP_VAL = 127
FORWARD_VAL = STOP_VAL + 23
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
        self.turn_timer = self.create_timer(3.35, self.stop)
        self.is_turning = False

    def send_velocity(self, feedback):
        message = MotorCurrents()
        if feedback.left_sensor > 15 and not self.is_turning: # left opening, turn left
            self.turn(message, "left", 3.35)
        elif feedback.front_sensor > 15 and not self.is_turning: # go forward
            message.left_wheels = FORWARD_VAL
            message.right_wheels = FORWARD_VAL
        elif feedback.right_sensor > 15 and not self.is_turning: # right opening, turn right
            self.turn(message, "right", 3.35)
        elif not self.is_turning: # turn around
            self.turn(message, "left", 3.35)
            self.turn(message, "left", 3.35)
        self.publisher.publish(message)

    def turn(self, message, dir, time):   
        if dir == "right":
            message.left_wheels = FORWARD_VAL
            message.right_wheels = BACKWARD_VAL
        else: # assume left
            message.left_wheels = BACKWARD_VAL
            message.right_wheels = FORWARD_VAL
        self.get_logger().info("Turning")
        self.publisher.publish(message)  
        self.is_turning = True
        if (self.turn_timer.is_canceled):
            self.turn_timer.reset()
        else:
            self.turn_timer = self.create_timer(time, self.stop)
            

    def stop(self):
        message = MotorCurrents()
        message.left_wheels = STOP_VAL
        message.right_wheels = STOP_VAL
        self.publisher.publish(message)
        self.turn_timer.cancel()
        self.is_turning = False

def main(args=None):
    rclpy.init(args=args)


    node = MotorControllerNode()


    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
