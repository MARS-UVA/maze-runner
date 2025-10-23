#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from serial_msgs.msg import MotorCurrents
from serial_msgs.msg import Feedback
import time


class SuperAwesomeAndRealNode(Node):


    def __init__(self):
        self.is_turning = False
        self.turn_timer = None
        super().__init__('testnode')
        self.publisher = self.create_publisher(
                msg_type=MotorCurrents,
                topic='motor_currents',
                qos_profile=10)


        self.feedback_subscriber_ = self.create_subscription(
            msg_type = Feedback,
            topic = 'feedback',
            qos_profile = 1,
            callback = self.wall_hugger
        )
        self.turn_timer = None
        self.is_turning = False

        self.get_logger().info("gorb")

    def send_velocity(self, feedback):
     
        message = MotorCurrents()
        #0 - 255, 127 = 0, 127 > forward, < 127 backwards
        
        distance_feedback = feedback.front_sensor
        left_feedback = feedback.left_sensor
        right_feedback = feedback.right_sensor

        # add code later when there are more sensors 

        if distance_feedback < 20 and not self.is_turning:
            self.turn(message, "left", 3.75)

   
      
       
    def wall_hugger(self, feedback):
        message = MotorCurrents()
        distance_feedback = feedback.front_sensor
        left_feedback = feedback.left_sensor
        if left_feedback > 30:
            self.turn(message, "left")
        elif distance_feedback > 30: 
            r_velo = 150
            l_velo = 150
            message.left_wheels = l_velo
            message.right_wheels = r_velo
            self.publisher.publish(message)  
        else:
            self.turn(message, "right")

    def turn(self, message, dir):   
        
       
        # I'm turning lefting it!
        if dir == "left":
            time.sleep(0.5)
            self.get_logger().info("I'm turning lefting it!")
            r_velo = 230
            l_velo = 50
        elif dir == "right":
            r_velo = 50
            l_velo = 230
        # if a direction isn't specified, stop.    
        else:
            r_velo = 127
            l_velo = 127

        message.left_wheels = l_velo
        message.right_wheels = r_velo
        
      
  
        self.publisher.publish(message)
        self.get_logger().info(f"Sent da message with {l_velo}, {r_velo}")
        self.is_turning = True
        time.sleep(0.67)
        if dir == "left":
            r_velo = 150
            l_velo = 150
            message.left_wheels = l_velo
            message.right_wheels = r_velo
            self.publisher.publish(message)
            self.get_logger().info("Going Forwards!")
            time.sleep(2)

   
        





def main(args=None):
    rclpy.init(args=args)


    node = SuperAwesomeAndRealNode()


    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()

        

if __name__ == '__main__':
    main()