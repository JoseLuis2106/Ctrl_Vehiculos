#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Path
import tf2_ros
import math

class PreciseParkingPI(Node):
    def __init__(self):
        super().__init__('precise_parking_pi')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.current_path = None
        self.target_idx = 0
        self.state = "IDLE"
        self.wait_timer = 0.0
        
        # --- PARÁMETROS DEL CONTROLADOR PI ---
        self.Kp = 5.5           # Ganancia Proporcional (Aumentada para más giro)
        self.Ki = 0.2           # Ganancia Integral (Para eliminar error acumulado)
        self.integral_error = 0.0
        self.max_integral = 0.6 # Anti-windup
        # -------------------------------------

        self.dist_tolerance = 0.12
        self.speed = 0.2
        self.create_timer(0.1, self.control_loop)

    def plan_callback(self, msg):
        if len(msg.poses) > 5:
            self.get_logger().info(f"Ruta recibida. Reseteando PI y arrancando.")
            self.current_path = msg
            self.target_idx = 0
            self.state = "DRIVING"
            self.integral_error = 0.0 # Resetear integral al inicio

    def control_loop(self):
        if self.state == "IDLE" or not self.current_path:
            return

        robot_tf = self.get_robot_pose()
        if not robot_tf: return

        if self.state == "ALIGNING_WHEELS":
            self.integral_error = 0.0 # Resetear al cambiar de rama
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.wait_timer > 2.0:
                self.state = "DRIVING"
            return

        target_pose = self.current_path.poses[self.target_idx].pose
        dist = self.get_dist(target_pose.position, robot_tf.transform.translation)

        if dist < self.dist_tolerance:
            if self.target_idx < len(self.current_path.poses) - 1:
                p_curr = self.transform_to_local(self.current_path.poses[self.target_idx].pose, robot_tf)
                p_next = self.transform_to_local(self.current_path.poses[self.target_idx+1].pose, robot_tf)
                
                if (p_curr.x * p_next.x) < 0: # Detección de pico Reeds-Shepp
                    self.stop_robot()
                    self.state = "ALIGNING_WHEELS"
                    self.wait_timer = self.get_clock().now().nanoseconds / 1e9
                self.target_idx += 1
            else:
                self.stop_robot()
                self.current_path = None
                self.state = "IDLE"
                return

        p_local = self.transform_to_local(target_pose, robot_tf)
        cmd = Twist()
        gear = 1.0 if p_local.x > 0 else -1.0
        cmd.linear.x = self.speed * gear

        # --- LÓGICA DEL CONTROLADOR PI ---
        # El error es el ángulo hacia el objetivo
        error = math.atan2(p_local.y, abs(p_local.x))
        
        # Acumular error integral solo si nos estamos moviendo
        self.integral_error += error * 0.1 
        
        # Anti-windup: Limitamos la influencia de la integral
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)
        
        # Salida del controlador
        u_p = self.Kp * error
        u_i = self.Ki * self.integral_error
        
        steering = u_p + u_i
        
        # Aplicamos el giro según el sentido de la marcha
        if gear > 0:
            cmd.angular.z = steering
        else:
            cmd.angular.z = -steering

        # Límite de seguridad para evitar que Gazebo ignore comandos extremos
        cmd.angular.z = max(min(cmd.angular.z, 2.0), -2.0)
        
        self.cmd_pub.publish(cmd)

    def get_robot_pose(self):
        try: return self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
        except: return None

    def get_dist(self, p1, p2):
        return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def transform_to_local(self, pose_world, robot_tf):
        dx = pose_world.position.x - robot_tf.transform.translation.x
        dy = pose_world.position.y - robot_tf.transform.translation.y
        q = robot_tf.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        lx = dx * math.cos(-yaw) - dy * math.sin(-yaw)
        ly = dx * math.sin(-yaw) + dy * math.cos(-yaw)
        return Point(x=lx, y=ly, z=0.0)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

def main():
    rclpy.init()
    node = PreciseParkingPI()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()