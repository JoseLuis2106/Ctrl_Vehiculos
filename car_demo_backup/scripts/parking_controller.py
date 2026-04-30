#!/usr/bin/env python3
"""
parking_controller.py  –  Controlador de aparcamiento para vehículo Ackermann (ROS 2 Humble)

Estrategia:
  1. Al recibir un path de Nav2 (Reeds-Shepp), se divide en segmentos entre cúspides.
  2. Para cada segmento se calcula la dirección de marcha (adelante / atrás) UNA SOLA VEZ,
     de forma que el controlador nunca necesita inferirla en tiempo real → elimina la oscilación.
  3. Al acercarse a una cúspide se reduce la velocidad proporcionalmente.
  4. Al llegar a una cúspide el coche frena y espera un breve instante (settling) antes de
     iniciar el segmento siguiente con la nueva dirección.
  5. Pure Pursuit clásico con ángulo de volante limitado a los valores físicos del Ackermann.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import numpy as np


# ──────────────────────────────────────────────────────────────────────────────
# Utilidades
# ──────────────────────────────────────────────────────────────────────────────

def pose_to_xy(pose_stamped):
    return np.array([pose_stamped.pose.position.x,
                     pose_stamped.pose.position.y])

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ──────────────────────────────────────────────────────────────────────────────
# Nodo principal
# ──────────────────────────────────────────────────────────────────────────────

class ParkingController(Node):

    # ── Parámetros ─────────────────────────────────────────────────────────────
    LOOKAHEAD_DIST     = 0.8    # [m]  distancia de mira para Pure Pursuit
    CUSP_TOLERANCE     = 0.2    # [m]  radio para considerar cúspide alcanzada
    GOAL_TOLERANCE     = 0.05   # [m]  radio para considerar goal final alcanzado
    MAX_SPEED          = 0.50   # [m/s]
    MIN_SPEED          = 0.15   # [m/s]  mínimo para no quedarse parado
    K_ANGULAR          = 5.0    # ganancia de dirección
    MAX_STEER          = 1.20   # [rad] límite físico del volante
    APPROACH_SLOW_DIST = 2.0    # [m]  empieza a frenar cuando queda esta distancia al cusp
    SETTLING_TIME      = 0.40   # [s]  pausa en cúspide para estabilizar

    def __init__(self):
        super().__init__('parking_controller')

        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path,     '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Estado del robot
        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        # Estado de navegación
        self.segments        = []   # lista de dicts: {'poses', 'direction'}
        self.current_seg_idx = 0
        self.is_navigating   = False
        self.settling        = False
        self.settle_start    = 0.0

        self.create_timer(0.05, self.control_loop)   # 20 Hz
        self.get_logger().info("Parking controller iniciado.")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.current_x   = msg.pose.pose.position.x
        self.current_y   = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        self._build_segments(msg.poses)
        if not self.segments:
            self.get_logger().warn("No se generaron segmentos; path ignorado.")
            return
        self.current_seg_idx = 0
        self.is_navigating   = True
        self.settling        = False
        self.get_logger().info(
            f"Nuevo path: {len(msg.poses)} puntos → {len(self.segments)} segmentos.")
        for i, s in enumerate(self.segments):
            d = "ADELANTE" if s['direction'] > 0 else "ATRÁS"
            self.get_logger().info(f"  Seg {i}: {len(s['poses'])} pts  [{d}]")

    # ── Construcción de segmentos ───────────────────────────────────────────────

    def _build_segments(self, poses):
        """
        Divide el path en segmentos separados por cúspides (cambios de sentido).
        Para cada segmento determina si es hacia adelante (+1) o hacia atrás (-1).
        """
        self.segments = []
        seg_start = 0

        for i in range(1, len(poses) - 1):
            v_before = pose_to_xy(poses[i])   - pose_to_xy(poses[i-1])
            v_after  = pose_to_xy(poses[i+1]) - pose_to_xy(poses[i])
            if np.dot(v_before, v_after) < 0:          # cúspide
                self._add_segment(poses[seg_start : i + 1])
                seg_start = i                           # la cúspide pertenece a ambos

        self._add_segment(poses[seg_start:])            # último segmento

    def _add_segment(self, poses):
        if len(poses) < 2:
            return
        direction = self._segment_direction(poses)
        self.segments.append({'poses': poses, 'direction': direction})

    def _segment_direction(self, poses):
        """
        Voto mayoritario: proyecta el movimiento de cada paso sobre la orientación
        del waypoint anterior para decidir si el segmento es adelante o atrás.
        """
        votes = 0
        for i in range(len(poses) - 1):
            dp    = pose_to_xy(poses[i+1]) - pose_to_xy(poses[i])
            yaw_i = yaw_from_quat(poses[i].pose.orientation)
            # Componente del movimiento en la dirección de la cabeza del coche
            proj  = dp[0] * math.cos(yaw_i) + dp[1] * math.sin(yaw_i)
            votes += 1 if proj >= 0 else -1
        return 1 if votes >= 0 else -1

    # ── Bucle de control ───────────────────────────────────────────────────────

    def control_loop(self):
        if not self.is_navigating or not self.segments:
            return

        # ── Segmento actual agotado / meta final ───────────────────────────────
        if self.current_seg_idx >= len(self.segments):
            self._publish_cmd(0.0, 0.0)
            self.is_navigating = False
            self.get_logger().info("¡Aparcamiento completado!")
            return

        # ── Fase settling (pausa en cúspide) ───────────────────────────────────
        if self.settling:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.settle_start
            if elapsed < self.SETTLING_TIME:
                self._publish_cmd(0.0, 0.0)
                return
            self.settling = False
            self.get_logger().info(
                f"Reanudando → segmento {self.current_seg_idx} "
                f"({'ADELANTE' if self.segments[self.current_seg_idx]['direction'] > 0 else 'ATRÁS'})")

        seg       = self.segments[self.current_seg_idx]
        poses     = seg['poses']
        direction = seg['direction']
        is_last   = (self.current_seg_idx == len(self.segments) - 1)

        # ── Distancias al path ─────────────────────────────────────────────────
        dists = [math.hypot(p.pose.position.x - self.current_x,
                            p.pose.position.y - self.current_y) for p in poses]
        closest_idx = int(np.argmin(dists))
        dist_to_end = dists[-1]

        # ── Detección de llegada ───────────────────────────────────────────────
        tol = self.GOAL_TOLERANCE if is_last else self.CUSP_TOLERANCE
        if dist_to_end < tol:
            self._publish_cmd(0.0, 0.0)
            if is_last:
                self.is_navigating = False
                self.get_logger().info("¡Meta final alcanzada!")
            else:
                self.get_logger().info(
                    f"Cúspide alcanzada (seg {self.current_seg_idx}). Settlingando...")
                self.current_seg_idx += 1
                self.settling     = True
                self.settle_start = self.get_clock().now().nanoseconds / 1e9
            return

        # ── Lookahead: nunca pasa del extremo del segmento actual ──────────────
        lookahead_idx = closest_idx
        for i in range(closest_idx, len(poses)):
            lookahead_idx = i
            if dists[i] >= self.LOOKAHEAD_DIST:
                break

        target = poses[lookahead_idx].pose.position

        # ── Pure Pursuit en frame local del robot ─────────────────────────────
        dx = target.x - self.current_x
        dy = target.y - self.current_y
        local_x =  dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
        local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)

        # En marcha atrás el punto de mira está "detrás": invertimos y lateral
        # para que la ley de dirección sea la misma.
        if direction < 0:
            local_y = -local_y

        # Ángulo de volante: atan2(y_local, |x_local|) escalado
        steer = self.K_ANGULAR * math.atan2(local_y, max(abs(local_x), 0.05))
        steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))

        # ── Velocidad adaptativa: frena al aproximarse al cusp ────────────────
        # factor va de 0 (muy cerca) a 1 (lejos)
        speed_factor = min(1.0, dist_to_end / self.APPROACH_SLOW_DIST)
        # Curva cuadrática suave para no frenar demasiado bruscamente
        speed_factor = speed_factor ** 1.5
        speed = self.MIN_SPEED + (self.MAX_SPEED - self.MIN_SPEED) * speed_factor

        self._publish_cmd(direction * speed, steer)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _publish_cmd(self, v: float, steer: float):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(steer)
        self.cmd_pub.publish(msg)


# ──────────────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    rclpy.spin(ParkingController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()