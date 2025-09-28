#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from serial_msgs.msg import MotorCurrents


class SuperAwesomeAndRealNode(Node):



 def __init__(self):
    super().__init__('testnode')

     self.publisher = self.create_publisher(
            msg_type=MotorCurrents,
            topic='motor_currents',
            qos_profile=10)


    self.feedback_subscriber_ = self.create_subscription(
        msg_type = feedback,
        topic = 'feedback',
        qos_profile = 1,
        callback = self.send_motor_currents
    )

     self.timer = self.create_timer(0.02, self.send_velocity)
     self.get_logger().info("gorb")

      def send_velocity(self):
        message = MotorCurrents()
        #0 - 255, 125 = 0, 127 > forward, < 127 backwards
        

        r_velo = 100
        l_velo = 220

        message.left_wheels = l_velo
        message.right_wheels = r_velo
        self.publisher.publish(message)   

def main(args=None):
    rclpy.init(args=args)


    node = SuperAwesomeAndRealNode()


    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
