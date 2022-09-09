import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class LaserSub(Node):
    def __init__(self):
        # Cria subscriber para scan
        super().__init__('lasersub')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Distância da frente : "%s"' % msg.ranges[0])

def main(args=None):
    rclpy.init(args=args)
    laser = LaserSub()
    while(True):
        rclpy.spin_once(laser)


if __name__ == '__main__':
    main()