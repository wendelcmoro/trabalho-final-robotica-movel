import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

PI = 3.1416

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

    def movimenta(self, i, t0):
        if i == 0:
            self.rotate(30, t0)
        else:
            self.andaFrente(t0)

    def rotate(self, angulo, t0):
        print("rotating!")
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

def main(args=None):
    rclpy.init(args=args)

    velocidade = VelocidadePub()

    i = 0
    while(True):
        t0 = time.perf_counter()
        velocidade.movimenta(i, t0)
        i = (i + 1) % 5000

if __name__ == '__main__':
    main()
