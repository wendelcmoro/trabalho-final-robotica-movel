import logging

from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .turtle_laser import LaserSub
import time


PI = 3.1416
FRONT_THRESHOLD = 0.25
LEFT_THRESHOLD = 0.2
CORNER_THRESHOLD = FRONT_THRESHOLD/1.5
SPEED = 0.1/2
ROTATION_SPEED = 0.1

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

class Turn(Enum):
    LEFT = ROTATION_SPEED
    RIGHT = -ROTATION_SPEED
    U = ROTATION_SPEED*4
    FORWARD = SPEED*PI


class State(Enum):
    GO_TO_WALL = 0
    FOLLOW_WALL = 1
    U_TURN = 2


class NinjaTurtle(Node):
    def __init__(self):
        # Criar publisher
        super().__init__('velocidadepub')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser = LaserSub()
        self.state = State.FOLLOW_WALL

    def main_loop(self):
        while True:
            rclpy.spin_once(self.laser)
            if self.state == State.GO_TO_WALL:
                self.go_to_wall()
            elif self.state == State.FOLLOW_WALL:
                self.follow_wall()
            elif self.state == State.U_TURN:
                self.u_turn()

    def go_to_wall(self):
        rclpy.spin_once(self.laser)

        old_distance = self.laser.norte
        while old_distance >= self.laser.norte:
            old_distance = self.laser.norte
            t0 = time.perf_counter()
            rclpy.spin_once(self.laser)
            if self.laser.oeste > self.laser.leste:
                if self.laser.oeste < FRONT_THRESHOLD:
                    break
                self.rotate(1, t0)
            else:
                self.rotate(-1, t0)

        rclpy.spin_once(self.laser)
        while self.laser.norte > FRONT_THRESHOLD:
            t0 = time.perf_counter()
            rclpy.spin_once(self.laser)
            self.andaFrente()
        self.log_laser()
        self.rotate(-90, t0)
        self.log_laser()

        self.state = State.FOLLOW_WALL

    def follow_wall(self):
        rclpy.spin_once(self.laser)
        t0 = time.perf_counter()

        if self.laser.norte < FRONT_THRESHOLD:
            self.rotate(-45, t0)
        elif self.laser.noroeste < CORNER_THRESHOLD:
            print("NO")
            self.rotate(-15, t0)
        elif self.laser.nordeste < CORNER_THRESHOLD:
            print("NE")
            self.rotate(15, t0)
        elif self.laser.oeste > LEFT_THRESHOLD + 0.02:
            if self.laser.oeste < LEFT_THRESHOLD*2:
                self.new_rotate(Turn.LEFT)
            else:
                self.state = State.U_TURN
            rclpy.spin_once(self.laser)
        elif self.laser.oeste < LEFT_THRESHOLD - 0.02:
            self.new_rotate(Turn.RIGHT)
            rclpy.spin_once(self.laser)
        else:
            self.andaFrente()

    def u_turn(self):
        if self.laser.oeste < LEFT_THRESHOLD:
            self.state = State.FOLLOW_WALL
        else:
            logging.debug(self.laser.oeste)
            self.new_rotate(Turn.FORWARD)

    def new_rotate(self, rotation_speed: Turn):
        logging.debug(f"Rotating to: {rotation_speed}")
        move_cmd = Twist()
        move_cmd.angular.z = rotation_speed.value
        move_cmd.linear.x = SPEED

        self.velocity_publisher.publish(move_cmd)


    def log_laser(self):
        rclpy.spin_once(self.laser)
        print("laser: ", [
            'N: ' + str(self.laser.norte),
            'NO: ' + str(self.laser.noroeste),
            'O: ' + str(self.laser.oeste),
            'S: ' + str(self.laser.sul),
            'L: ' + str(self.laser.leste),
            'NE: ' + str(self.laser.nordeste),
        ])


    def movimenta(self, t0):
        rclpy.spin_once(self.laser)

        if self.laser.norte > FRONT_THRESHOLD:
            self.andaFrente()
        else:
             self.rotate(-90, t0)


    def rotate(self, angulo, t0, auto_stop = True):
        print(f"rodando {angulo} graus...")
        if auto_stop:
            self.parar()


        vel_rotacao = 15 # valor qualquer
        vel_angular = round((vel_rotacao*PI)/180, 1)
        angulo_alvo = round((angulo*PI)/180, 1)

        move_cmd = Twist()
        move_cmd.linear.x = 0.0 if auto_stop else SPEED
        move_cmd.angular.z = abs(vel_angular) if angulo > 0 else -abs(vel_angular)
        angulo_atual = 0.0
        # Publica mesma mensagem ate chegar no angulo desejado
        print(angulo_alvo, angulo_atual, vel_angular)
        while angulo_atual < abs(angulo_alvo):
            self.velocity_publisher.publish(move_cmd)
            t1 = time.perf_counter()
            angulo_atual = abs(vel_angular)*(t1-t0)

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.velocity_publisher.publish(move_cmd)


    def andaFrente(self):
        logging.debug("Moving forward")
        move_cmd = Twist()
        move_cmd.linear.x = SPEED
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        # self.get_logger().info('Andando para frente com velocidade SPEED')

    def parar(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        self.get_logger().info('Parada total.')

def main(args=None):
    rclpy.init(args=args)

    michellangelo = NinjaTurtle()
    michellangelo.main_loop()



if __name__ == '__main__':
    main()
