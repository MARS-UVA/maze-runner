#!/usr/bin/env python3
import time
import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from serial_msgs.msg import MotorCurrents, Feedback

STOP_VAL = 127
FORWARD_VAL = STOP_VAL + 23
BACKWARD_VAL = STOP_VAL - 27
LEFT_DIST = 30
RIGHT_DIST = 30
FRONT_DIST = 15
TURN_TIME = 3.35

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
        if feedback.left_sensor > LEFT_DIST: # left opening, turn left
            self.turn(message, "left", TURN_TIME)
        elif feedback.front_sensor > FRONT_DIST: # go forward
            message.left_wheels = FORWARD_VAL
            message.right_wheels = FORWARD_VAL
            self.publisher.publish(message)
        elif feedback.right_sensor > RIGHT_DIST: # right opening, turn right
            self.turn(message, "right", TURN_TIME)
        else: # turn around
            self.turn(message, "left", TURN_TIME)
            self.turn(message, "left", TURN_TIME)
        if feedback.front_sensor > FRONT_DIST: # go forward
            message.left_wheels = FORWARD_VAL
            message.right_wheels = FORWARD_VAL
            self.publisher.publish(message)
            time.sleep(0.5)

    def turn(self, message, dir, duration):   
        if dir == "right":
            message.left_wheels = FORWARD_VAL
            message.right_wheels = FORWARD_VAL
        else: # assume left
            message.left_wheels = FORWARD_VAL
            message.right_wheels = FORWARD_VAL
        self.get_logger().info("Turning")
        self.publisher.publish(message)
        time.sleep(duration)
            

    def stop(self):
        message = MotorCurrents()
        message.left_wheels = STOP_VAL
        message.right_wheels = STOP_VAL
        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)


    node = MotorControllerNode()


    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
