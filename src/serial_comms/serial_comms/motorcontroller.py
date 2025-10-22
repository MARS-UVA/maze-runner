#!/usr/bin/env python3
import time
import rclpy #type: ignore
from rclpy.node import Node #type: ignore
from serial_msgs.msg import MotorCurrents, Feedback

STOP_VAL = 127
FORWARD_VAL = STOP_VAL + 50
BACKWARD_VAL = STOP_VAL - 50
LEFT_DIST = 30
RIGHT_DIST = 30
FRONT_DIST = 15
TURN_TIME = 1.3
FORWARD_TIME = 2
PAUSE_TIME = 1

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
            self.turn(message, "left")
            self.pause(message)
            if feedback.front_sensor > FRONT_DIST: # go forward
                self.move_forward(message)
                self.pause(message)
        elif feedback.front_sensor > FRONT_DIST: # go forward
            message.left_wheels = FORWARD_VAL
            message.right_wheels = FORWARD_VAL
            self.publisher.publish(message)
        elif feedback.right_sensor > RIGHT_DIST: # right opening, turn right
            self.turn(message, "right")
            self.pause(message)
            if feedback.front_sensor > FRONT_DIST: # go forward
                self.move_forward(message)
                self.pause(message)
        else: # turn around
            self.turn(message, "left")
            self.pause(message)
            self.turn(message, "left")
            self.pause(message)
            if feedback.front_sensor > FRONT_DIST: # go forward
                self.move_forward(message)
                self.pause(message)

    def turn(self, message, dir):   
        if dir == "right":
            message.left_wheels = FORWARD_VAL
            message.right_wheels = BACKWARD_VAL
        else: # assume left
            message.left_wheels = BACKWARD_VAL
            message.right_wheels = FORWARD_VAL
        self.get_logger().info("Turning")
        self.publisher.publish(message)
        time.sleep(TURN_TIME)
            
    def move_forward(self, message):
        message.left_wheels = FORWARD_VAL
        message.right_wheels = FORWARD_VAL
        self.publisher.publish(message)
        time.sleep(FORWARD_TIME)

    def pause(self, message):
        message.left_wheels = STOP_VAL
        message.right_wheels = STOP_VAL
        self.publisher.publish(message)
        time.sleep(PAUSE_TIME)

def main(args=None):
    rclpy.init(args=args)


    node = MotorControllerNode()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
