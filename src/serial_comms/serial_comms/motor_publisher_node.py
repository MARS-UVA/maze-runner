#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
# Import the custom message type your serial_node uses
from serial_msgs.msg import MotorCurrents, Feedback

class MotorPublisherNode(Node):

    def __init__(self):
        # 1. Change the node name to be descriptive
        super().__init__('motor_publisher_node')
        
        # 2. Change the publisher to use the MotorCurrents message type
        #    and the 'motor_currents' topic.
        self.publisher = self.create_publisher(
            msg_type=MotorCurrents,
            topic='motor_currents',
            qos_profile=10)
        
        self.feedback_subscriber = self.create_subscription(
            msg_type = Feedback,
            topic='feedback',
            qos_profile=1,
            callback = self.send_motor_command
        )
            
        self.get_logger().info("Motor publisher node started. Sending commands...")

    def send_motor_command(self, feedback):
        # 3. Create a MotorCurrents message instead of a Twist message.
        message = MotorCurrents()
        if feedback.us_sensor <60:
            message.left_wheels = 127
            message.right_wheels = 127
        else:
        # 4. Set the wheel current values. The serial_node expects integers.
        #    127 is still, >127 is forward, <127 is backward.
            #    Let's make it move forward.
            message.left_wheels = 150 
            message.right_wheels = 150
            
        self.publisher.publish(message)
        self.get_logger().info(f'Publishing: left={message.left_wheels}, right={message.right_wheels}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()