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
        

    def turn_left(self, message):
        # turn velocities
        r_velo, l_velo = 230, 70
        self.get_logger().info(f"jsadhfjhsdjafhjdahsfjhdjkfhdjshafjdkshfjkashdkfjhdsjkakf: glob")
        # publish
        message.right_wheels = r_velo
        message.left_wheels = l_velo
        self.publisher.publish(message)
        # stop listening to sensors for duration of the turn (2sec)
        time.sleep(2/3)
        # go forward after turning left
        message.right_wheels = 160
        message.left_wheels = 160
        self.publisher.publish(message)
        

    def turn_right(self, message):
        # turn velocities
        r_velo, l_velo = 70, 230
        self.get_logger().info(f"jsadhfjhsdjafhjdahsfjhdjkfhdjshafjdkshfjkashdkfjhdsjkakf: glob")
        # publish
        message.right_wheels = r_velo
        message.left_wheels = l_velo
        self.publisher.publish(message)
        # stop listening to sensors for duration of the turn (2sec)
        time.sleep(2/3)
    

    def turn_around(self, message):
        # turn velocities
        r_velo, l_velo = 70, 230
        self.get_logger().info(f"jsadhfjhsdjafhjdahsfjhdjkfhdjshafjdkshfjkashdkfjhdsjkakf: glob")
        # publish
        message.right_wheels = r_velo
        message.left_wheels = l_velo
        self.publisher.publish(message)
        # stop listening to sensors for duration of the turn (2sec)
        time.sleep(1)

    def send_velocity(self, feedback):
        message = MotorCurrents()
        r_velo = 160
        l_velo = 160

        if feedback.left_sensor > 30:
            self.turn_left(message)
        elif feedback.front_sensor >30:
            r_velo = 160
            l_velo = 160
        elif feedback.right_sensor >30:
            self.turn_right(message)
        elif feedback.left_sensor > 30 and feedback.front_sensor >30 and feedback.right_sensor >30:
            r_velo = 127
            l_velo = 127
        else:
            self.turn_around(message)



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

