#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from serial_msgs.msg import MotorCurrents
from serial_msgs.msg import Feedback
import time


class SuperAwesomeAndRealNode(Node):



    def __init__(self):
        super().__init__('testnode')
        self.publisher = self.create_publisher(
                msg_type=MotorCurrents,
                topic='motor_currents',
                qos_profile=10)


        self.feedback_subscriber_ = self.create_subscription(
            msg_type = Feedback,
            topic = 'feedback',
            qos_profile = 1,
            callback = self.send_velocity
        )
        self.turn_timer = None
        self.is_turning = False

        self.get_logger().info("gorb")

    def send_velocity(self, feedback):
        message = MotorCurrents()
        #0 - 255, 127 = 0, 127 > forward, < 127 backwards
        
        distance_feedback = feedback.us_sensor

        # add code later when there are more sensors 

     

        if distance_feedback < 20 and not self.is_turning:
            self.turn_left(message)

           # r_velo = 127
           # l_velo = 127
      
       
      

    def turn_left(self, message):   
        r_velo = 150
        l_velo = 100
        message.left_wheels = l_velo
        message.right_wheels = r_velo
        
        self.get_logger().info("I'm turning lefting it!")
        self.get_logger().info(message)
        #self.get_logger().info()
        self.publisher.publish(message)  
        self.is_turning = True
        self.turn_timer = self.create_timer(3, self.stop)
        

    def stop(self):
        message = MotorCurrents()
        r_velo = 127
        l_velo = 127
        message.left_wheels = l_velo
        message.right_wheels = r_velo
        self.publisher.publish(message)  
        self.turn_timer.cancel()
        self.is_turning = False
        self.turn_timer = None



def main(args=None):
    rclpy.init(args=args)


    node = SuperAwesomeAndRealNode()


    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
