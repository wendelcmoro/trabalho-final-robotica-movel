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

    def main_loop(self):
        while (True):
            t0 = time.perf_counter()
            self.movimenta(t0)

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

    def follow_wall(self):
        # last_last_wall_distance = self.laser.oeste
        rclpy.spin_once(self.laser)
        # already_rotated = 0
        last_oeste_wall_distance = self.laser.oeste
        last_leste_wall_distance = self.laser.leste
        while True:
            rclpy.spin_once(self.laser)
            t0 = time.perf_counter()

            is_going_left = last_oeste_wall_distance < self.laser.oeste# or last_leste_wall_distance > self.laser.leste
            is_going_right = last_oeste_wall_distance > self.laser.oeste# or last_leste_wall_distance < self.laser.leste
            print(f"OO: {last_oeste_wall_distance}, OL: {last_leste_wall_distance}")
            print(f"CO: {self.laser.oeste}, CL: {self.laser.leste}")
            print(f"L: {is_going_left}. R: {is_going_right}")

            if self.laser.norte < FRONT_THRESHOLD:
                self.rotate(-45, t0)
            elif self.laser.noroeste < CORNER_THRESHOLD:
                print("NO")
                self.rotate(-15, t0)
            elif self.laser.nordeste < CORNER_THRESHOLD:
                print("NE")
                self.rotate(15, t0)
            elif self.laser.oeste > LEFT_THRESHOLD + 0.02 and not is_going_left:
                # adjusting_value = abs(1 - last_oeste_wall_distance/self.laser.oeste) * 1000
                print("E", last_oeste_wall_distance, self.laser.oeste, "f", LEFT_THRESHOLD + 0.02)
                if self.laser.oeste < FRONT_THRESHOLD*2:
                    self.rotate(3, t0, False)
                    # self.rotate(max(min(adjusting_value, 8), 3), t0, False)
                else:
                    # self.rotate(max(min(adjusting_value, 20), 3), t0, False)
                    self.rotate(12, t0, False)
                rclpy.spin_once(self.laser)
            elif self.laser.oeste < LEFT_THRESHOLD - 0.02 and not is_going_right:
                # adjusting_value = (1 - last_oeste_wall_distance/self.laser.oeste) * 1000
                print("D", last_oeste_wall_distance, self.laser.oeste, "f", LEFT_THRESHOLD - 0.02)
                self.rotate(-3, t0, False)
                # self.rotate(min(max(adjusting_value, -8), -3), t0, False)
                rclpy.spin_once(self.laser)
                print(last_oeste_wall_distance, self.laser.oeste, "f", LEFT_THRESHOLD - 0.02)
            else:
                print("moving")

                self.andaFrente()
            last_oeste_wall_distance = self.laser.oeste
            last_leste_wall_distance = self.laser.leste


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

    velocidade = VelocidadePub()

    velocidade.go_to_wall()

    velocidade.follow_wall()


if __name__ == '__main__':
    main()
