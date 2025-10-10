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
            callback = self.send_velocity
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

           # r_velo = 127
           # l_velo = 127
      
       
    def wall_hugger(self, feedback):
        message = MotorCurrents()
        distance_feedback = feedback.front_sensor
        left_feedback = feedback.left_sensor
        if left_feedback > 20:
            self.turn(message, "left", 3.25)
        elif distance_feedback > 20: 
            r_velo = 150
            l_velo = 150
            message.left_wheels = l_velo
            message.right_wheels = r_velo
            self.publisher.publish(message)  
        else:
            self.turn(message, "right", 3.25)

    def turn(self, message, dir, duration):   
        if dir == "left":
            r_velo = 150
            l_velo = 100
        else:
            r_velo = 100
            l_velo = 150
        message.left_wheels = l_velo
        message.right_wheels = r_velo
        
        self.get_logger().info("I'm turning lefting it!")
        #self.get_logger().info()
        self.publisher.publish(message)
        self.get_logger().info(f"Sent da message with {l_velo}, {r_velo}")
        self.is_turning = True
        time.sleep(10)
        r_velo = 150
        l_velo = 150
        message.left_wheels = l_velo
        message.right_wheels = r_velo
        self.publisher.publish(message)
        self.get_logger().info("Going Forwards!")
        time.sleep(5)
        #self.turn_timer = self.create_timer(duration, self.stop(dir))
    
    def stop(self, dir):
        message = MotorCurrents()
        if dir == "left":
            r_velo = 150
            l_velo = 150
            message.left_wheels = l_velo
            message.right_wheels = r_velo
            self.publisher.publish(message)
            self.destroy_timer(self.turn_timer)
            self.turn_timer = self.create_timer(2, self.stop("forward"))
        r_velo = 127
        l_velo = 127
        message.left_wheels = l_velo
        message.right_wheels = r_velo
        self.publisher.publish(message)  
        self.destroy_timer(self.turn_timer)
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