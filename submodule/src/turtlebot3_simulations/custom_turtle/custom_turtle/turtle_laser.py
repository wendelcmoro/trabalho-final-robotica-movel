import rclpy
from rclpy.node import Node
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
        # checar se precisa iniciar no init
        self.norte = msg.ranges[0]
        self.noroeste = msg.ranges[45]
        self.oeste = msg.ranges[90]
        self.sul = msg.ranges[180]
        self.leste = msg.ranges[270]
        self.nordeste = msg.ranges[315]
        
def main(args=None):
    rclpy.init(args=args)

    laser = LaserSub()

    i = 0
    while(i < 10):
        rclpy.spin_once(laser)
        print("Distância do norte: " + str(laser.norte))
        i += 1

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
