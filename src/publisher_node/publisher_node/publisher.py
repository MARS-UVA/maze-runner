import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, Duration
from serial_msgs.msg import MotorCurrents
from serial_msgs.msg import Feedback
import time

class SampleNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.publisher = self.create_publisher(
            msg_type=MotorCurrents,
            topic='motor_currents', 
            qos_profile=10)

        
        self.feedback_subscriber = self.create_subscription(
            msg_type=Feedback,
            topic='feedback',
            qos_profile=1, 
            callback=self.send_velocity)
        

    def turn_left(self, feedback):

        # turn velocities
        r_velo, l_velo = 230, 70
        self.get_logger().info(f"jsadhfjhsdjafhjdahsfjhdjkfhdjshafjdkshfjkashdkfjhdsjkakf: glob")
        # stop listening to sensors for duration of the turn (2sec)
        time.sleep(2)
        # move straight velocites
        r_velo, l_velo = 160, 160
        return r_velo, l_velo
        

    def turn_right(self):

        # turn velocities
        r_velo, l_velo = 70, 230
        self.get_logger().info(f"jsadhfjhsdjafhjdahsfjhdjkfhdjshafjdkshfjkashdkfjhdsjkakf: glob")
        # stop listening to sensors for duration of the turn (2sec)
        time.sleep(2)
        # move straight velocites
        r_velo, l_velo = 160, 160
        return r_velo, l_velo
    

    def send_velocity(self, feedback):
        message = MotorCurrents()
        #initial velocities
        r_velo = 160
        l_velo = 160

        # turns left if there is a wall 30cm ahead
        if feedback.front_sensor < 30:
            self.turn_left(feedback)


        message.right_wheels = r_velo
        message.left_wheels = l_velo
        self.get_logger().info(f"jsadhfjhsdjafhjdahsfjhdjkfhdjshafjdkshfjkashdkfjhdsjkakf: VELOCITY!!!")

        self.publisher.publish(message)





    
def main(args=None):
    rclpy.init(args=args)

    node = SampleNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

