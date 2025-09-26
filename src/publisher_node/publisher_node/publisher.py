import rclpy
from rclpy.node import Node, QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy, Duration
from serial_msgs.msg import MotorCurrents


class SampleNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.publisher = self.create_publisher(
            msg_type=MotorCurrents,
            topic='motor_currents', qos_profile=10)
        self.timer = self.create_timer(0.5, self.send_velocity)
        
    def send_velocity(self):
        message = MotorCurrents()
        message.left_wheels = 150
        message.right_wheels = 150
        self.publisher.publish(message)


    
def main(args=None):
    rclpy.init(args=args)

    node = SampleNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

