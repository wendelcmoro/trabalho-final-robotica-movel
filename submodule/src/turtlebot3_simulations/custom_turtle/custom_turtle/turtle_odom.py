import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OdomSub(Node):
    def __init__(self):
        # Cria subscriber para scan
        super().__init__('odomsub')
        self.subscription = self.create_subscription(
            String,
            'odom',
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.subscription

    def listener_callback(self, msg):
        print('odom: ', msg)
