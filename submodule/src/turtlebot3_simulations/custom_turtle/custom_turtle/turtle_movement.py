import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .turtle_laser import LaserSub

from enum import Enum
import time
import logging
import signal
import sys

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! Ending program')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

PI = 3.1416
FRONT_THRESHOLD = 0.25
LEFT_THRESHOLD = 0.15
CORNER_THRESHOLD = FRONT_THRESHOLD/1.5
SPEED = 0.1
ROTATION_SPEED = 1.0

logging.basicConfig(format='%(levelname)s:%(message)s', level=logging.DEBUG)

class Turn(Enum):
    LEFT = ROTATION_SPEED
    RIGHT = -ROTATION_SPEED * 0.5

class State(Enum):
    FOWARD = 0
    LEFT = 2
    RIGHT = 3
    FAST_FOWARD = 5
    STOPPED = 10


class NinjaTurtle(Node):
    def __init__(self):
        # Criar publisher
        super().__init__('velocidadepub')
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.laser = LaserSub()
        self.state = State.FOWARD
        self.next_state = State.FOWARD

    def main_loop(self):
        while True:
            # Loggin
            print("state:", self.state, end=' | ')
            self.log_laser()

            # Generate next state
            if (self.state == State.LEFT):
                self.next_state = State.FOWARD
            else: 
                self.set_next_state()

            # Move to next state
            self.state, self.next_state = self.next_state, State.STOPPED

            # Take action
            if self.state == State.STOPPED:
                self.parar()
            elif self.state == State.FOWARD:
                self.andaFrente(SPEED)
            elif self.state == State.FAST_FOWARD:
                self.andaFrente(SPEED*1)
            elif self.state == State.LEFT:
                self.rotate(Turn.LEFT)
            elif self.state == State.RIGHT:
                self.rotate(Turn.RIGHT)
            
    def set_next_state(self):
        rclpy.spin_once(self.laser)

        if self.laser.norte < 0.25:
            self.next_state = State.RIGHT
        elif self.laser.noroeste < 0.25:
            self.next_state = State.RIGHT
        elif self.laser.oeste > 0.2:
            self.next_state = State.LEFT
        elif self.state == State.FOWARD or self.state == State.FAST_FOWARD:
            self.next_state = State.FAST_FOWARD
        else:
            self.next_state = State.FOWARD
        return

    def log_laser(self):
        rclpy.spin_once(self.laser)
        
        values = [['N: ', self.laser.norte],
            ['NO: ', self.laser.noroeste],
            ['O: ', self.laser.oeste],
            ['S: ', self.laser.sul],
            ['L: ', self.laser.leste],
            ['NE: ', self.laser.nordeste]]
        
        print("laser: ", end="")
        for val in values:
            print(val[0], "{:.4f}".format(val[1]), end=" ")
        print()

    def andaFrente(self, speed):
        move_cmd = Twist()
        move_cmd.linear.x = speed
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        # self.get_logger().info('Andando para frente com velocidade ' + str(SPEED))
    
    def rotate(self, rotation_speed: Turn):
        # logging.debug(f"Rotating to: {rotation_speed}")
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = rotation_speed.value

        self.velocity_publisher.publish(move_cmd)

    def parar(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        # self.get_logger().info('Parada total.')


def main(args=None):
    rclpy.init(args=args)

    michellangelo = NinjaTurtle()
    michellangelo.main_loop()



if __name__ == '__main__':
    main()
