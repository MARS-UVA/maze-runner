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


        self.get_logger().info("gorb")

    def send_velocity(self, feedback):
        message = MotorCurrents()
        #0 - 255, 127 = 0, 127 > forward, < 127 backwards
        
        distance_feedback = feedback.us_sensor

        # add code later when there are more sensors 

     

        if distance_feedback < 15:

            r_velo = 150
            l_velo = 100
            message.left_wheels = l_velo
            message.right_wheels = r_velo
            self.publisher.publish(message)


            self.get_logger().info("I'm boutta turn it!")
            #self.turn_left(message)
            self.get_logger().info("I'm done turning it!")
        else: 
            r_velo = 127
            l_velo = 127
            message.left_wheels = l_velo
            message.right_wheels = r_velo
            self.publisher.publish(message) 
            self.get_logger().info("I'm stopped")
            self.get_logger().info(message.left_wheels)
            self.get_logger().info(message.right_wheels)

           # r_velo = 127
           # l_velo = 127
      
       
      

    def turn_left(self, message):   
        start_time = time.time()
        while time.time() - start_time < 3:
            r_velo = 150
            l_velo = 100
            message.left_wheels = l_velo
            message.right_wheels = r_velo
            self.publisher.publish(message)
        
        r_velo = 127
        l_velo = 127
        message.left_wheels = l_velo
        message.right_wheels = r_velo
        self.get_logger().info("I'm turning lefting it!")
        #self.get_logger().info(message)
        #self.get_logger().info()
        self.publisher.publish(message)  

        return r_velo, l_velo





def main(args=None):
    rclpy.init(args=args)


    node = SuperAwesomeAndRealNode()


    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
