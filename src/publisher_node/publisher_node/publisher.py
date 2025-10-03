import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, Duration
from serial_msgs.msg import MotorCurrents
from serial_msgs.msg import Feedback

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
        s#elf.timer = self.create_timer(0.5, self.send_velocity)

    def turn_left(self, feedback):

        r_velo = 154
        l_velo = 100
        self.get_logger().info(f"jsadhfjhsdjafhjdahsfjhdjkfhdjshafjdkshfjkashdkfjhdsjkakf: glob")
        
        return r_velo, l_velo
        
    def send_velocity(self, feedback):
        message = MotorCurrents()
        r_velo = 150
        l_velo = 150

        # stops the robot if there is a wall 30cm ahead
        # if feedback.us_sensor < 30:
        #     r_velo = 127
        #     l_velo = 127

        # turns the robot left if there is a wall 30cm ahead
        if feedback.us_sensor < 30:
            r_velo, l_velo = self.turn_left(feedback)

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

