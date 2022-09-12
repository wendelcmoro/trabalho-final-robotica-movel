import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class VelocidadePub(Node):
    def __init__(self):
        # Criar publisher
        super().__init__('velocidadepub')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.movimenta)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0  
        self.vel_msg.linear.y = 0.0 
        self.vel_msg.linear.z = 0.0  
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0  
        self.vel_msg.angular.z = 0.0

    def movimenta(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        self.get_logger().info('Andando para frente com velocidade 0.1')


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
        self.norte = msg.ranges[0]


def main(args=None):
    rclpy.init(args=args)

    laser = LaserSub()
    velocidade = VelocidadePub()

    while(True):
        rclpy.spin_once(laser)
        print("Distância da parede: ", str(laser.norte))
        velocidade.movimenta()
            
if __name__ == '__main__':
    main()
