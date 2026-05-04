#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2 
import math
import numpy as np


# ═════════════════════════════════════════════════════════════════════════════
# Parámetros
# ═════════════════════════════════════════════════════════════════════════════

LOOKAHEAD_DIST      = 0.8
CUSP_TOLERANCE      = 0.20
GOAL_TOLERANCE      = 0.20
GOAL_YAW_TOL        = 0.03   # ~1.7°
MAX_SPEED           = 0.70
MIN_SPEED           = 0.15
MICRO_SPEED         = 0.08
K_ANGULAR           = 6.5
K_YAW_ALIGN         = 7.5
MAX_STEER           = 1.20
APPROACH_SLOW_DIST  = 2.5
ALIGN_TRIGGER_DIST  = 4.0
SETTLING_TIME       = 0.80
LASER2FRONT         = 2.79
LASER2BACK          = 2.0

# ── Fase final ───────────────────────────────────────────────────────────────
PF_ENTRY_DIST       = 0.55   # [m]  distancia para activar el latch
PF_DIR_DEADBAND     = 0.05   # [m]  zona muerta para no cambiar dir si local_x ≈ 0
PF_FLIP_TICKS       = 3      # ciclos de stop al detectar cambio de dirección


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

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(Path, '/plan',  self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom',  self.odom_callback, 10)
        self.laser_sub = self.create_subscription(PointCloud2, '/prius/center_laser/scan', self.scan_callback, 10)

        self.current_x   = 0.0
        self.current_y   = 0.0
        self.current_yaw = 0.0

        self.segments        = []
        self.current_seg_idx = 0
        self.is_navigating   = False
        self.settling        = False
        self.settle_start    = 0.0
        self.goal_yaw        = 0.0
        self.goal_x          = 0.0
        self.goal_y          = 0.0

        # Fase final
        self.pf_active       = False
        self.pf_direction    = 1      # última dirección conocida (con deadband)
        self.pf_flip_count   = 0      # ciclos de parada al cambiar dirección

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Parking controller iniciado.")

    # ─────────────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.current_x   = msg.pose.pose.position.x
        self.current_y   = msg.pose.pose.position.y
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)


    def scan_callback(self, msg):
        self.min_fwd = np.Inf
        self.min_bwd = np.Inf
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            dist = math.sqrt(p[0]**2 + p[1]**2)
            # Filtro de altura y distancia para limpiar ruido
            if -0.1 < p[2] < 0.1 and 0.1 < dist < 12.0:
                if p[0] > LASER2FRONT and abs(p[1]) < 1.1 and dist - LASER2FRONT < self.min_fwd:
                    self.min_fwd = dist - LASER2FRONT
                elif p[0] < -LASER2BACK and abs(p[1]) < 1.1 and dist - LASER2BACK < self.min_bwd:
                    self.min_bwd = dist - LASER2BACK

        # if self.min_fwd != np.Inf or self.min_bwd != np.Inf:
        #     self.get_logger().info(f"min_fwd: {self.min_fwd:.2f}, min_bwd: {self.min_bwd:.2f}")


    def path_callback(self, msg):
        if len(msg.poses) < 2:
            return
        last = msg.poses[-1].pose
        self.goal_yaw = yaw_from_quat(last.orientation)
        self.goal_x   = last.position.x
        self.goal_y   = last.position.y
        self._build_segments(msg.poses)
        if not self.segments:
            self.get_logger().warn("Sin segmentos; path ignorado.")
            return
        self.current_seg_idx = 0
        self.is_navigating   = True
        self.settling        = False
        self.pf_active       = False
        self.pf_direction    = 1
        self.pf_flip_count   = 0
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

        # ── Latch Fase final ──────────────────────────────────────────────
        if is_last and not self.pf_active and dist_to_end < PF_ENTRY_DIST:
            self.pf_active = True
            self.get_logger().info(f"[PF] ACTIVADA dist={dist_to_end:.2f}m")

        if self.pf_active:
            self._phase3(dist_to_end)
            return

        # ── Cúspide ───────────────────────────────────────────────────────
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

        if direction > 0 and self.min_fwd < 0.07 or direction < 0 and self.min_bwd < 0.07:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info("Movimiento imposible. Choque inminente.")
            return
        
        self._publish_cmd(direction * speed, steer)

    # ─────────────────────────────────────────────────────────────────────────
    # Fase final – Corrección de yaw
    # ─────────────────────────────────────────────────────────────────────────

    def _phase3(self, dist_to_end: float):
        yaw_error = normalize_angle(self.goal_yaw - self.current_yaw)

        # ── Parada breve al cambiar dirección ─────────────────────────────
        if self.pf_flip_count > 0:
            self._publish_cmd(0.0, 0.0)
            self.pf_flip_count -= 1
            return

        # ── Condición de fin ──────────────────────────────────────────────
        if abs(yaw_error) <= GOAL_YAW_TOL and dist_to_end <= GOAL_TOLERANCE:
            self._publish_cmd(0.0, 0.0)
            self.is_navigating = False
            self.get_logger().info(
                f"✓ Aparcamiento completado | "
                f"dist={dist_to_end:.3f}m | yaw_err={math.degrees(yaw_error):.1f}°")
            return

        # ── Dirección dinámica: proyectar coche→goal sobre la cabeza ──────
        dx_goal  = self.goal_x - self.current_x
        dy_goal  = self.goal_y - self.current_y
        local_x  = (dx_goal * math.cos(self.current_yaw)
                    + dy_goal * math.sin(self.current_yaw))

        # if abs(local_x) > PF_DIR_DEADBAND:
        #     new_dir = 1 if local_x > 0.0 else -1
        #     if new_dir != self.pf_direction:
        #         # Cambio de dirección detectado → parada breve
        #         self.pf_direction  = new_dir
        #         self.pf_flip_count = PF_FLIP_TICKS
        #         self._publish_cmd(0.0, 0.0)
        #         self.get_logger().info(
        #             f"[PF] Cambio dir → {'FWD' if new_dir > 0 else 'REV'} "
        #             f"(local_x={local_x:.3f})")
        #         return

        if self.min_fwd < 0.1 or self.min_bwd < 0.1:
            new_dir = 1 if self.min_bwd < 0.1 else -1
            if new_dir != self.pf_direction:
                # Cambio de dirección detectado → parada breve
                self.pf_direction  = new_dir
                self.pf_flip_count = PF_FLIP_TICKS
                self._publish_cmd(0.0, 0.0)
                self.get_logger().info(
                    f"[PF] Cambio dir → {'FWD' if new_dir > 0 else 'REV'} "
                    f"(min_fwd={self.min_fwd:.3f})"
                    f"(min_bwd={self.min_bwd:.3f})")
                return

        # ── Corregir yaw ──────────────────────────────────────────────────
        steer = clamp(K_YAW_ALIGN * yaw_error, -MAX_STEER, MAX_STEER)
        if self.pf_direction > 0 and self.min_fwd < 0.07 or self.pf_direction < 0 and self.min_bwd < 0.07:
            self._publish_cmd(0.0, 0.0)
            self.get_logger().info("Movimiento imposible. Choque inminente.")
            return
        
        self._publish_cmd(self.pf_direction * MICRO_SPEED, steer)
        self.get_logger().debug(
            f"[PF] dir={'F' if self.pf_direction > 0 else 'R'} "
            f"yaw_err={math.degrees(yaw_error):.1f}° "
            f"steer={math.degrees(steer):.1f}° "
            f"lx={local_x:.3f} dist={dist_to_end:.2f}")

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