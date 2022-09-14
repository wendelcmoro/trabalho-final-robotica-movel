import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .turtle_laser import LaserSub
import time

PI = 3.1416
FRONT_THRESHOLD = 0.2

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

        self.laser = LaserSub()

        self.virando = False

    def movimenta(self, t0):
        print("t0: ", t0)
        rclpy.spin_once(self.laser)
        print("laser: ", [
            'N: '  + str(self.laser.norte),
            'NO: ' + str(self.laser.noroeste),
            'O: '  + str(self.laser.oeste),
            'S: '  + str(self.laser.sul),
            'L: '  + str(self.laser.leste),
            'NE: ' + str(self.laser.nordeste),
        ])

        if (self.laser.norte > FRONT_THRESHOLD):
            self.andaFrente(t0)
        elif (self.virando):
            self.rotate(90, t0)
            self.virando = False
        else:
            self.parar(t0)
            self.virando = True

    def rotate(self, angulo, t0):
        print(f"rodando {angulo} graus...")
        vel_rotacao = 15 # valor qualquer
        vel_angular = round((vel_rotacao*PI)/180, 1)
        angulo_alvo = round((angulo*PI)/180, 1)

        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = abs(vel_angular)
        angulo_atual = 0.0
        # Publica mesma mensagem ate chegar no angulo desejado
        while (angulo_atual < angulo_alvo):
            self.velocity_publisher.publish(move_cmd)
            t1 = time.perf_counter()
            angulo_atual = vel_angular*(t1-t0)

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.velocity_publisher.publish(move_cmd)


    def andaFrente(self, t0):
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        self.get_logger().info('Andando para frente com velocidade 0.1')

    def parar(self, t0):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        self.get_logger().info('Parada total.')

def main(args=None):
    rclpy.init(args=args)

    velocidade = VelocidadePub()

    while(True):
        t0 = time.perf_counter()
        velocidade.movimenta(t0)

if __name__ == '__main__':
    main()
