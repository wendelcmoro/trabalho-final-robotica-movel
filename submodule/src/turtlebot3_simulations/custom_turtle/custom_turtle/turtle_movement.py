import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .turtle_laser import LaserSub
from .turtle_odom import OdomSub
import time
from enum import Enum
from std_msgs.msg import String

class Status(Enum):
    PROCURANDO_PAREDE   = 'PROCURANDO_PAREDE'
    ACOMPANHANDO_PAREDE = 'ACOMPANHANDO_PAREDE'

class Naveg(Enum):
    FRENTE_LIVRE   = 'FRENTE_LIVRE'
    OESTE_LIVRE    = 'OESTE_LIVRE'
    LESTE_LIVRE    = 'LESTE_LIVRE'
    EVITAR_COLISAO = 'EVITAR_COLISAO'

PI = 3.1416
FRONT_THRESHOLD = 0.23999
VEL_ROT = 15
VEL_FRONT = 0.085
TIMER_PERIOD = 0.05
FREQ_PRINT = 5

class VelocidadePub(Node):
    def __init__(self):
        # Criar publisher
        super().__init__('velocidadepub')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.movimenta)
        self.vel_msg = Twist()
        self.vel_msg.linear.x = 0.0  
        self.vel_msg.linear.y = 0.0 
        self.vel_msg.linear.z = 0.0  
        self.vel_msg.angular.x = 0.0
        self.vel_msg.angular.y = 0.0  
        self.vel_msg.angular.z = 0.0

        self.laser = LaserSub()
        self.odom = OdomSub()
        self.rotacionando = False

        self.estado = Status.PROCURANDO_PAREDE
        self.naveg = Naveg.FRENTE_LIVRE

    def movimenta(self, i, t0):
        rclpy.spin_once(self.laser)

        if (self.rotacionando):
            return

        # 1. Procura qualquer parede
        if (self.estado == Status.PROCURANDO_PAREDE):
            # vai para frente até achar alguma parede
            if (self.laser.norte > FRONT_THRESHOLD):
                self.naveg = Naveg.FRENTE_LIVRE
                self.andaFrente(i, t0)
            else:
                self.rotate(90, t0)
                self.estado = Status.ACOMPANHANDO_PAREDE
        # 2. Tenta acompanhar uma parede encontrada
        elif (self.estado == Status.ACOMPANHANDO_PAREDE):
            # verifica se vai colidir com a parede
            if (self.laser.norte <= FRONT_THRESHOLD):
                self.naveg = Naveg.EVITAR_COLISAO
                self.rotate(90, t0)

            elif (self.laser.nordeste <= FRONT_THRESHOLD):
                self.naveg = Naveg.EVITAR_COLISAO
                self.rotate(5, t0)

            elif (self.laser.noroeste <= FRONT_THRESHOLD):
                self.naveg = Naveg.EVITAR_COLISAO
                self.rotate(-5, t0)

            # verifica se a parede ainda está ao leste
            elif (self.laser.leste > FRONT_THRESHOLD * 1.5):
                self.naveg = Naveg.LESTE_LIVRE
                self.andaFrente(i, t0)
                t1 = time.perf_counter()
                # espera alguns ciclos
                while (t1 - t0 <= 3):
                    t1 = time.perf_counter()
                self.parar(t0)

                self.rotate(-90, t0)

                self.estado = Status.PROCURANDO_PAREDE

            # vai para frente, acompanhando a parede
            elif (self.laser.norte > FRONT_THRESHOLD):
                self.naveg = Naveg.FRENTE_LIVRE
                self.andaFrente(i, t0)
        else:
            self.parar(t0)

        print(f"{self.estado} | {self.naveg}")

        if (i % FREQ_PRINT == 0):
            print(f"i: {i}")
            print("laser: ")
            # formatar floar com 2 casas decimais
            print(f" NO  N  NE     {self.laser.noroeste:.2f} {self.laser.norte:.2f} {self.laser.nordeste:.2f} ")
            print(f"  O  C  L      {self.laser.oeste:.2f}  C   {self.laser.leste:.2f}")

    def rotate(self, angulo, t0):
        if (self.rotacionando):
            return

        print(f"rodando {angulo} graus...")
        self.parar(t0)
        self.rotacionando = True
        vel_rotacao = VEL_ROT # valor qualquer
        vel_angular = round((vel_rotacao*PI)/180, 1)
        angulo_alvo = round((angulo*PI)/180, 1)

        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = abs(vel_angular) if angulo > 0 else -abs(vel_angular)
        angulo_atual = 0.0
        # Publica mesma mensagem ate chegar no angulo desejado
        while angulo_atual < abs(angulo_alvo):
            self.velocity_publisher.publish(move_cmd)
            t1 = time.perf_counter()
            angulo_atual = abs(vel_angular)*(t1-t0)

        self.parar(t0)

        self.rotacionando = False


    def andaFrente(self, i, t0):
        move_cmd = Twist()
        move_cmd.linear.x = VEL_FRONT
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        if (i % FREQ_PRINT == 0):
            print('Andando para frente com velocidade 0.1')

    def parar(self, t0):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        print('Parada total.')

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
