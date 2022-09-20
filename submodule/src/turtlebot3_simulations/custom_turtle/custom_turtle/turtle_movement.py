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
    SMALL_LEFT = ROTATION_SPEED * 0.01
    # SMALL_RIGHT = ROTATION_SPEED * 0.5

class State(Enum):
    FOWARD = 0
    FOWARD_LEFT = 1
    LEFT = 2
    RIGHT = 3
    SMALL_LEFT = 4
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

        rclpy.spin_once(self.laser)
        self.last_lidar = self.copyLaserData()

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
                self.andaFrente(SPEED*1)
            elif self.state == State.FAST_FOWARD:
                self.andaFrente(SPEED*1)
            elif self.state == State.FOWARD_LEFT:
                self.new_rotate(Turn.LEFT)
            elif self.state == State.LEFT:
                self.rotate(Turn.LEFT)
            elif self.state == State.RIGHT:
                self.rotate(Turn.RIGHT)
            elif self.state == State.SMALL_LEFT:
                self.rotate(Turn.SMALL_LEFT)

    def set_next_state(self):
        rclpy.spin_once(self.laser)

        oeste = self.laser.oeste 
        last_oeste = self.last_lidar["oeste"]
        noroeste = self.laser.noroeste 
        last_noroeste = self.last_lidar["noroeste"]

        if self.laser.norte < 0.25:
            self.next_state = State.RIGHT
        elif self.laser.noroeste < 0.25:
            self.next_state = State.RIGHT
        elif self.laser.oeste > 0.2: # and self.laser.oeste < self.laser.noroeste: #and self.laser.oeste < self.laser.noroeste*2/3:
            self.next_state = State.LEFT
        elif self.state == State.FOWARD or self.state == State.FAST_FOWARD:
            self.next_state = State.FAST_FOWARD
        else:
            self.next_state = State.FOWARD
        return

        if (self.state == State.LEFT):
            if self.laser.oeste > 0.25:
                if self.laser.oeste < self.last_lidar["oeste"]:
                    print("Take left after left")
                    self.next_state = State.FOWARD_LEFT
                else:
                    self.next_state = State.FOWARD
            else:
                self.next_state = State.FOWARD
        else:
            if self.laser.oeste > 0.25:
                print("Start left")
                self.next_state = State.LEFT
            else:
                self.next_state = State.FOWARD


        self.last_lidar = self.copyLaserData()

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
        # logging.debug("Moving forward")
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

    def new_rotate(self, rotation_speed: Turn):
        # logging.debug(f"Rotating to: {rotation_speed}")
        move_cmd = Twist()
        move_cmd.linear.x = SPEED
        move_cmd.angular.z = rotation_speed.value

        self.velocity_publisher.publish(move_cmd)

    def parar(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0

        self.velocity_publisher.publish(move_cmd)
        # self.get_logger().info('Parada total.')

    def copyLaserData(self):
        return {
            "norte": self.laser.norte,
            "noroeste": self.laser.noroeste,
            "oeste": self.laser.oeste,
            "sul": self.laser.sul,
            "leste": self.laser.leste,
            "nordeste": self.laser.nordeste,
        }


def main(args=None):
    rclpy.init(args=args)

    michellangelo = NinjaTurtle()
    michellangelo.main_loop()



if __name__ == '__main__':
    main()
