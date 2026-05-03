#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
import math
import numpy as np


# ═════════════════════════════════════════════════════════════════════════════
# Parámetros
# ═════════════════════════════════════════════════════════════════════════════

LOOKAHEAD_DIST      = 0.8    # [m]
CUSP_TOLERANCE      = 0.20   # [m]
GOAL_TOLERANCE      = 0.20   # [m]  radio posición para «en meta»
GOAL_YAW_TOL        = 0.03   # [rad] ~1.72°
MAX_SPEED           = 0.70   # [m/s]
MIN_SPEED           = 0.15   # [m/s]
MICRO_SPEED         = 0.08   # [m/s]
K_ANGULAR           = 6.0
K_YAW_ALIGN         = 7.5
MAX_STEER           = 1.20   # [rad]
APPROACH_SLOW_DIST  = 2.5    # [m]
ALIGN_TRIGGER_DIST  = 4.0    # [m]
SETTLING_TIME       = 0.80   # [s]  pausa entre cúspides (fases 1-2)

# ── Fase 3 ────────────────────────────────────────────────────────────────
P3_ENTRY_DIST       = 0.55   # [m]  distancia para activar el latch de Fase 3
P3_DRIFT_TOL_FACTOR = 1.5    # umbral de alejamiento = factor × GOAL_TOLERANCE
P3_SETTLE_TICKS     = 8      # ciclos de parada limpia (~0.4 s a 20 Hz)


# ═════════════════════════════════════════════════════════════════════════════
# Estados Fase 3
# ═════════════════════════════════════════════════════════════════════════════

P3_APPROACH       = "APPROACH"
P3_SETTLE_TO_YAW  = "SETTLE_TO_YAW"
P3_YAW_CORRECT    = "YAW_CORRECT"
P3_SETTLE_TO_APPR = "SETTLE_TO_APPROACH"
P3_DONE           = "DONE"


# ═════════════════════════════════════════════════════════════════════════════
# Helpers
# ═════════════════════════════════════════════════════════════════════════════

def pose_to_xy(ps):
    return np.array([ps.pose.position.x, ps.pose.position.y])

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def normalize_angle(a):
    while a >  math.pi: a -= 2.0 * math.pi
    while a <= -math.pi: a += 2.0 * math.pi
    return a

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ═════════════════════════════════════════════════════════════════════════════
# Nodo
# ═════════════════════════════════════════════════════════════════════════════

class ParkingController(Node):

    def __init__(self):
        super().__init__('parking_controller')

        self.cmd_pub  = self.create_publisher(Twist,     '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path,     '/plan',  self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom',  self.odom_callback, 10)

        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        self.segments        = []
        self.current_seg_idx = 0
        self.is_navigating   = False
        self.settling        = False
        self.settle_start    = 0.0
        self.goal_yaw        = 0.0

        # Fase 3
        self.p3_active       = False        # latch: True una vez que se entra en Fase 3
        self.p3_state        = P3_YAW_CORRECT
        self.p3_settle_count = 0

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Parking controller iniciado.")

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.current_x   = msg.pose.pose.position.x
        self.current_y   = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        self.goal_yaw = yaw_from_quat(msg.poses[-1].pose.orientation)
        self._build_segments(msg.poses)
        if not self.segments:
            self.get_logger().warn("Sin segmentos; path ignorado.")
            return
        self.current_seg_idx = 0
        self.is_navigating   = True
        self.settling        = False
        # Reset completo de Fase 3 al recibir un path nuevo
        self.p3_active       = False
        self.p3_state        = P3_APPROACH
        self.p3_settle_count = 0
        self.get_logger().info(
            f"Nuevo path: {len(msg.poses)} pts → {len(self.segments)} segs "
            f"| goal_yaw={math.degrees(self.goal_yaw):.1f}°")
        for i, s in enumerate(self.segments):
            d = "ADELANTE" if s['direction'] > 0 else "ATRÁS"
            self.get_logger().info(f"  Seg {i}: {len(s['poses'])} pts [{d}]")

    # ─────────────────────────────────────────────────────────────────────────
    # Segmentos
    # ─────────────────────────────────────────────────────────────────────────

    def _build_segments(self, poses):
        self.segments = []
        seg_start = 0
        for i in range(1, len(poses) - 1):
            v_before = pose_to_xy(poses[i])   - pose_to_xy(poses[i-1])
            v_after  = pose_to_xy(poses[i+1]) - pose_to_xy(poses[i])
            if np.dot(v_before, v_after) < 0:
                self._add_segment(poses[seg_start : i + 1])
                seg_start = i
        self._add_segment(poses[seg_start:])

    def _add_segment(self, poses):
        if len(poses) < 2:
            return
        self.segments.append({'poses': poses,
                               'direction': self._segment_direction(poses)})

    def _segment_direction(self, poses):
        votes = 0
        for i in range(len(poses) - 1):
            dp    = pose_to_xy(poses[i+1]) - pose_to_xy(poses[i])
            yaw_i = yaw_from_quat(poses[i].pose.orientation)
            proj  = dp[0] * math.cos(yaw_i) + dp[1] * math.sin(yaw_i)
            votes += 1 if proj >= 0 else -1
        return 1 if votes >= 0 else -1

    # ─────────────────────────────────────────────────────────────────────────
    # Bucle principal (20 Hz)
    # ─────────────────────────────────────────────────────────────────────────

    def control_loop(self):
        if not self.is_navigating or not self.segments:
            return

        if self.current_seg_idx >= len(self.segments):
            self._publish_cmd(0.0, 0.0)
            self.is_navigating = False
            self.get_logger().info("¡Aparcamiento completado!")
            return

        # ── Settling entre cúspides ───────────────────────────────────────
        if self.settling:
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self.settle_start
            if elapsed < SETTLING_TIME:
                self._publish_cmd(0.0, 0.0)
                return
            self.settling = False
            d = "ADELANTE" if self.segments[self.current_seg_idx]['direction'] > 0 else "ATRÁS"
            self.get_logger().info(f"→ Seg {self.current_seg_idx} [{d}]")

        seg       = self.segments[self.current_seg_idx]
        poses     = seg['poses']
        direction = seg['direction']
        is_last   = (self.current_seg_idx == len(self.segments) - 1)

        dists       = [math.hypot(p.pose.position.x - self.current_x,
                                   p.pose.position.y - self.current_y)
                       for p in poses]
        closest_idx = int(np.argmin(dists))
        dist_to_end = dists[-1]

        # ── Activación del latch de Fase 3 ────────────────────────────────
        if is_last and not self.p3_active and dist_to_end < P3_ENTRY_DIST:
            self.p3_active = True
            # Elegir estado inicial: si ya está en posición, ir directo a YAW
            if dist_to_end <= P3_DRIFT_TOL_FACTOR * GOAL_TOLERANCE:
                self.p3_state = P3_SETTLE_TO_YAW
                self.p3_settle_count = P3_SETTLE_TICKS
            else:
                self.p3_state = P3_APPROACH
            self.get_logger().info(
                f"[P3] ACTIVADA (dist={dist_to_end:.2f}m) → {self.p3_state}")

        # ── Delegar a Fase 3 si está activa ───────────────────────────────
        if self.p3_active:
            self._phase3(dist_to_end, direction, poses)
            return

        # ── Cúspide intermedia ────────────────────────────────────────────
        if not is_last and dist_to_end < CUSP_TOLERANCE:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info(f"Cúspide (seg {self.current_seg_idx}). Settling…")
            self.current_seg_idx += 1
            self.settling     = True
            self.settle_start = self.get_clock().now().nanoseconds / 1e9
            return

        # ── Lookahead ─────────────────────────────────────────────────────
        lookahead_idx = closest_idx
        for i in range(closest_idx, len(poses)):
            lookahead_idx = i
            if dists[i] >= LOOKAHEAD_DIST:
                break

        target  = poses[lookahead_idx].pose.position
        dx = target.x - self.current_x
        dy = target.y - self.current_y
        local_x =  dx * math.cos(self.current_yaw) + dy * math.sin(self.current_yaw)
        local_y = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)
        if direction < 0:
            local_y = -local_y

        steer_pp = K_ANGULAR * math.atan2(local_y, max(abs(local_x), 0.05))

        # Fase 2: mezcla PP + yaw
        if is_last and dist_to_end <= ALIGN_TRIGGER_DIST:
            yaw_error = normalize_angle(self.goal_yaw - self.current_yaw)
            steer_yaw = clamp(K_YAW_ALIGN * yaw_error * direction, -MAX_STEER, MAX_STEER)
            alpha     = max(0.0, 1.0 - dist_to_end / ALIGN_TRIGGER_DIST) ** 0.5
            steer     = (1.0 - alpha) * steer_pp + alpha * steer_yaw
        else:
            steer = steer_pp

        steer        = clamp(steer, -MAX_STEER, MAX_STEER)
        speed_factor = clamp(dist_to_end / APPROACH_SLOW_DIST, 0.0, 1.0) ** 1.5
        speed        = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * speed_factor
        self._publish_cmd(direction * speed, steer)

    # ─────────────────────────────────────────────────────────────────────────
    # Fase 3 – máquina de estados
    # ─────────────────────────────────────────────────────────────────────────

    def _phase3(self, dist_to_end: float, direction: int, poses):
        yaw_error  = normalize_angle(self.goal_yaw - self.current_yaw)
        drift_tol  = P3_DRIFT_TOL_FACTOR * GOAL_TOLERANCE

        # ── SETTLE (parada limpia) ─────────────────────────────────────────
        if self.p3_state in (P3_SETTLE_TO_YAW, P3_SETTLE_TO_APPR):
            self._publish_cmd(0.0, 0.0)
            self.p3_settle_count -= 1
            if self.p3_settle_count <= 0:
                next_state = (P3_YAW_CORRECT if self.p3_state == P3_SETTLE_TO_YAW
                              else P3_APPROACH)
                self.p3_state = next_state
                self.get_logger().info(f"[P3] → {next_state}")
            return

        # ── YAW_CORRECT ───────────────────────────────────────────────────
        if self.p3_state == P3_YAW_CORRECT:
            # Condición de fin: posición Y orientación ok
            if abs(yaw_error) <= GOAL_YAW_TOL and dist_to_end <= GOAL_TOLERANCE:
                self._publish_cmd(0.0, 0.0)
                self.is_navigating = False
                self.p3_state      = P3_DONE
                self.get_logger().info(
                    f"✓ Aparcamiento completado | "
                    f"dist={dist_to_end:.3f}m | yaw_err={math.degrees(yaw_error):.1f}°")
                return

            # Se alejó demasiado: volver a acercarse
            if dist_to_end > drift_tol:
                self.p3_state        = P3_SETTLE_TO_APPR
                self.p3_settle_count = P3_SETTLE_TICKS
                self.get_logger().info(
                    f"[P3] YAW → SETTLE_TO_APPROACH "
                    f"(dist={dist_to_end:.2f} > {drift_tol:.2f})")
                self._publish_cmd(0.0, 0.0)
                return

            # Corregir yaw in-situ
            steer = clamp(K_YAW_ALIGN * yaw_error, -MAX_STEER, MAX_STEER)
            self._publish_cmd(MICRO_SPEED, steer)
            self.get_logger().debug(
                f"[YAW] err={math.degrees(yaw_error):.1f}° "
                f"steer={math.degrees(steer):.1f}° dist={dist_to_end:.2f}")
            return

        # ── APPROACH ──────────────────────────────────────────────────────
        if self.p3_state == P3_APPROACH:
            # Calcular dirección dinámica al goal
            goal_pos = poses[-1].pose.position
            dx = goal_pos.x - self.current_x
            dy = goal_pos.y - self.current_y
            local_x_goal = (dx * math.cos(self.current_yaw)
                            + dy * math.sin(self.current_yaw))
            appr_dir = 1 if local_x_goal >= 0.0 else -1

            if dist_to_end <= GOAL_TOLERANCE:
                self.p3_state        = P3_SETTLE_TO_YAW
                self.p3_settle_count = P3_SETTLE_TICKS
                self.get_logger().info(
                    f"[P3] APPROACH → SETTLE_TO_YAW (dist={dist_to_end:.2f})")
                self._publish_cmd(0.0, 0.0)
                return

            self._publish_cmd(appr_dir * MICRO_SPEED, 0.0)
            self.get_logger().debug(
                f"[APPR] dist={dist_to_end:.2f} dir={appr_dir}")
            return

    # ─────────────────────────────────────────────────────────────────────────

    def _publish_cmd(self, v: float, steer: float):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(steer)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ParkingController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
