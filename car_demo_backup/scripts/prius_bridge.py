#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from prius_msgs.msg import Control

class PriusBridge(Node):
    def __init__(self):
        super().__init__('prius_bridge')
        self.pub = self.create_publisher(Control, '/prius/control', 10)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)

    def callback(self, msg):
        ctrl = Control()
        # Ajuste de giro: el Prius gira de -1.0 a 1.0. 
        # Si Nav2 gira poco, sube este divisor (ej. 0.6)
        ctrl.steer = msg.angular.z / 0.5 
        
        if abs(msg.linear.x) < 0.05:
            ctrl.throttle = 0.0
            ctrl.brake = 10.0      # Frenado total
            ctrl.shift_gears = 1  # NEUTRAL
        else:
            ctrl.brake = 0.0
            ctrl.throttle = 0.1   # Velocidad de parking controlada
            if msg.linear.x > 0:
                ctrl.shift_gears = 2 # DRIVE
            else:
                ctrl.shift_gears = 3 # REVERSE
        
        self.pub.publish(ctrl)

def main():
    rclpy.init()
    rclpy.spin(PriusBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()