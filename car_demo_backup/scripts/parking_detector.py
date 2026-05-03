#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import PoseStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2 
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from action_msgs.msg import GoalStatus
import numpy as np
import colorsys
import math
import tf2_ros

class ParkingDetector(Node):
    def __init__(self):
        super().__init__('parking_detector')

        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_viz = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        
        # self.subscription = self.create_subscription(LaserScan, '/prius/center_laser/scan', self.scan_callback, qos_lidar)
        self.subscription = self.create_subscription(PointCloud2, '/prius/center_laser/scan', self.scan_callback, qos_lidar)
        self.marker_pub = self.create_publisher(Marker, '/parking_slot_marker', qos_viz)
        self.cluster_pub = self.create_publisher(MarkerArray, '/viz_clusters', qos_viz)
        self.goal_pub = self.create_publisher(PoseStamped, '/parking_goal', qos_viz)        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')     # Con parking_controller

        # Configuración de TF2 para transformar coordenadas
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.min_slot_width = 5.5
        self.max_slot_width = 9.0 
        self.offsetx = 1.4 
        self.offsety = 1.3

        self.replan_timer = None
        self.parking_finished = False
        
        self.state = "SEARCHING"
        self.path_requested = False
        self.get_logger().info("Nodo parking_detector iniciado.")

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.8 
        self.cmd_pub.publish(msg)

    def stop_car(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.cmd_pub.publish(msg)
        self.state = "STOPPED"
        self.get_logger().info("¡Hueco rebasado! Coche detenido.")

    def send_path_request(self, goal_pose_odom):
        """Envía la meta transformada en ODOM a la acción de Nav2"""
        if self.path_requested:
            return

        # Corregido: nombre del cliente coincidente con __init__
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Acción NavigateToPose no disponible')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose_odom
        
        self.get_logger().info('Enviando meta definitiva a Nav2...')
        self.path_requested = True
        
        # Conectamos el callback para recibir la respuesta del servidor
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    # def publish_goal(self, p1, p2, center):
    #     goal_laser = PoseStamped()
    #     goal_laser.header.frame_id = "center_laser_link" 
    #     goal_laser.header.stamp = self.get_clock().now().to_msg()
    #     yaw_calle = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    #     inward_angle = yaw_calle - (math.pi / 2.0)
        
    #     # dist_offset = 1.5 

    #     # goal_laser.pose.position.x = float(center[0] + dist_offset * math.cos(inward_angle))
    #     # goal_laser.pose.position.y = float(center[1] + dist_offset * math.sin(inward_angle))
    #     goal_laser.pose.position.x = float(center[0])
    #     goal_laser.pose.position.y = float(center[1])
        
    #     goal_laser.pose.orientation.z = math.sin(yaw_calle / 2.0)
    #     goal_laser.pose.orientation.w = math.cos(yaw_calle / 2.0)

    #     try:
    #         goal_odom = self.tf_buffer.transform(goal_laser, "odom", timeout=rclpy.duration.Duration(seconds=0.2))
    #         goal_odom.pose.position.z = 0.0
            
    #         self.get_logger().info(f"Publicando Goal en ODOM: x={goal_odom.pose.position.x:.2f}, y={goal_odom.pose.position.y:.2f}")
    #         self.goal_pub.publish(goal_odom)
    #         self.send_path_request(goal_odom)
            
    #     except Exception as e:
    #         self.get_logger().error(f"Error transformando goal: {str(e)}")

    def publish_goal(self, center, yaw):
        try:            
            goal_laser = PoseStamped()
            goal_laser.header.frame_id = "center_laser_link"
            goal_laser.header.stamp = rclpy.time.Time().to_msg() 
            
            goal_laser.pose.position.x = float(center[0]) - self.offsetx
            goal_laser.pose.position.y = float(center[1])

            goal_laser.pose.orientation.z = math.sin(yaw / 2.0)
            goal_laser.pose.orientation.w = math.cos(yaw / 2.0)

            self.goal_odom = self.tf_buffer.transform(goal_laser, "odom", timeout=rclpy.duration.Duration(seconds=0.1))
            self.goal_odom.pose.position.z = 0.0

            self.get_logger().info(f"Goal publicado en ODOM:")
            self.get_logger().info(f"x={self.goal_odom.pose.position.x:.2f}")
            self.get_logger().info(f"y={self.goal_odom.pose.position.y:.2f}")
            self.get_logger().info(f"z={self.goal_odom.pose.orientation.z:.2f}")
            self.get_logger().info(f"w={self.goal_odom.pose.orientation.w:.2f}")
            self.goal_pub.publish(self.goal_odom)

            # self.send_path_request(self.goal_odom)     # Sin parking_controller


            # Con parking_controller
            goal_msg = ComputePathToPose.Goal()
            goal_msg.goal = PoseStamped()
            goal_msg.goal.header.frame_id = "odom"
            goal_msg.goal.pose = self.goal_odom.pose
            goal_msg.planner_id = "GridBased" # Debe coincidir con nav2_params.yaml

            self.planner_client.wait_for_server()
            self._send_goal_future = self.planner_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

        except Exception as e:
            self.get_logger().error(f"Error al publicar el goal: {str(e)}")


    def request_replan(self):
        if not self.parking_finished:
            self.goal_pub.publish(self.goal_odom)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Planificación rechazada por Nav2')
            self.path_requested = False
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        if self.replan_timer is None:                                           #Con parking_controller
            self.replan_timer = self.create_timer(2.0, self.request_replan)


    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("¡Meta planificada con éxito!")
        else:
            self.get_logger().warn(f"La meta falló con estado: {status}")

    # def scan_callback(self, msg):
    #     if self.state != "STOPPED":
    #         self.move_forward()        
    
    #     ranges = np.array(msg.ranges)
    #     angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
    #     mask = (ranges > 0.1) & (ranges < 10.0)
    #     if not np.any(mask): return
        
    #     points = np.column_stack((ranges[mask] * np.cos(angles[mask]), ranges[mask] * np.sin(angles[mask])))

    #     # Clustering
    #     dist_to_sensor = np.linalg.norm(points, axis=1)
    #     depth_diff = np.abs(dist_to_sensor[1:] - dist_to_sensor[:-1])
    #     spatial_diff = np.linalg.norm(points[1:] - points[:-1], axis=1)
    #     indices = np.where((spatial_diff > 0.3) | (depth_diff > 0.4))[0] + 1
    #     clusters = np.split(points, indices)
    #     clusters = [c for c in clusters if len(c) > 3]
        
    #     self.viz_clusters(clusters)

    #     if len(clusters) < 3: return

    #     for i in range(len(clusters) - 2):
    #         c_left, c_mid, c_right = clusters[i], clusters[i+1], clusters[i+2]
    #         d_left = np.min(np.linalg.norm(c_left, axis=1))
    #         d_mid = np.min(np.linalg.norm(c_mid, axis=1))
    #         d_right = np.min(np.linalg.norm(c_right, axis=1))

    #         if d_mid > (d_left + 0.5) and d_mid > (d_right + 0.5):
    #             p1 = c_left[np.argmin(np.linalg.norm(c_left, axis=1))]
    #             p2 = c_right[np.argmin(np.linalg.norm(c_right, axis=1))]
    #             width = np.linalg.norm(p2 - p1)

    #             if self.min_slot_width < width < self.max_slot_width and self.state != "STOPPED":
    #                 center = (p1 + p2) / 2
    #                 self.publish_marker(center, width)

    #                 if center[0] < -0.5: 
    #                     self.stop_car()
    #                     self.publish_goal(p1, p2, center)
    #                 else:
    #                     self.move_forward()

    def scan_callback(self, msg):
        if self.state != "STOPPED":
            self.move_forward()        

        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            dist = math.sqrt(p[0]**2 + p[1]**2)
            # Filtro de altura y distancia para limpiar ruido
            if -0.1 < p[2] < 0.1 and 0.1 < dist < 12.0:
                points_list.append([p[0], p[1]])

        if not points_list:
            return
        
        points = np.array(points_list)

        angles = np.arctan2(points[:, 1], points[:, 0])
        points = points[np.argsort(angles)]

        dist_to_sensor = np.linalg.norm(points, axis=1)
        depth_diff = np.abs(dist_to_sensor[1:] - dist_to_sensor[:-1])
        spatial_diff = np.linalg.norm(points[1:] - points[:-1], axis=1)
        
        indices = np.where((spatial_diff > 0.3) | (depth_diff > 0.4))[0] + 1
        clusters = np.split(points, indices)
        clusters = [c for c in clusters if len(c) > 5]
        
        self.viz_clusters(clusters)

        if len(clusters) < 3: 
            return

        for i in range(len(clusters) - 2):
            c_left, c_mid, c_right = clusters[i], clusters[i+1], clusters[i+2]

            y_left = np.mean(np.abs(c_left[:, 1]))
            y_mid = np.mean(np.abs(c_mid[:, 1]))
            y_right = np.mean(np.abs(c_right[:, 1]))

            if y_mid > (y_left + 1.5) and y_mid > (y_right + 1.5):
                d_left = np.linalg.norm(c_left, axis=1)
                d_mid = np.linalg.norm(c_mid, axis=1)
                d_right = np.linalg.norm(c_right, axis=1)
                p1 = c_left[np.argmin(d_left)]
                p2 = c_right[np.argmin(d_right)]
                p3 = c_right[0]
                p4 = c_left[-1]
                
                width = np.linalg.norm(p2 - p1)

                if self.min_slot_width < width < self.max_slot_width and self.state != "STOPPED":
                    center = (p1 + p2 + p3 + p4) / 4.0
                    self.publish_parking_visual(p1, p2, p3, p4, width)

                    y_wall = np.mean(c_mid[:, 1])                    
                    y_goal = center[1]
                    d_goal_wall = abs(y_wall - y_goal)

                    # --- CORRECCIÓN OFFSET Y (Profundidad) ---
                    # Si el muro está a la izquierda (y_wall > y_goal), restamos para alejarnos.
                    # Si está a la derecha, sumamos.
                    direction_y = -1.0 if y_wall > y_goal else 1.0
                    y_adjustment = max(0, self.offsety - d_goal_wall)
                    center[1] += direction_y * y_adjustment

                    if center[0] < 0.0: 
                        self.get_logger().info(f"HUECO DETECTADO! Ancho: {width:.2f}m")
                        self.stop_car()
                        pts = [p1, p2, p3, p4]
                        pts_sorted = sorted(pts, key=lambda p: p[0])
                        
                        p_rear_avg = (pts_sorted[0] + pts_sorted[1]) / 2.0
                        p_front_avg = (pts_sorted[2] + pts_sorted[3]) / 2.0
                        
                        v = p_front_avg - p_rear_avg
                        yaw = math.atan2(v[1], v[0])
                        
                        self.get_logger().info(f"Yaw calculado: {yaw:.2f} rad")
                        self.publish_goal(center, yaw)


    def viz_clusters(self, clusters):
        marker_array = MarkerArray()
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = "center_laser_link" 
            marker.id = i
            marker.type = Marker.SPHERE_LIST
            marker.scale.x = marker.scale.y = marker.scale.z = 0.1
            rgb = colorsys.hsv_to_rgb(i / max(len(clusters), 1), 1.0, 1.0)
            marker.color.r, marker.color.g, marker.color.b = rgb
            marker.color.a = 0.7
            for p in cluster:
                point = Point()
                point.x, point.y, point.z = float(p[0]), float(p[1]), 0.0
                marker.points.append(point)
            marker_array.markers.append(marker)
        self.cluster_pub.publish(marker_array)

    # def publish_marker(self, pos, width):
    #     marker = Marker()
    #     marker.header.frame_id = "center_laser_link" 
    #     marker.type = Marker.TEXT_VIEW_FACING
    #     marker.text = f"PARKING: {width:.2f}m"
    #     marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(pos[0]), float(pos[1]), 1.0
    #     marker.scale.z = 0.4
    #     marker.color.a, marker.color.g = 1.0, 1.0
    #     self.marker_pub.publish(marker)

    def publish_parking_visual(self, p1, p2, p3, p4, width):
        marker = Marker()
        marker.header.frame_id = "center_laser_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.LINE_STRIP
        marker.id = 200
        marker.scale.x = 0.1 # Grosor
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0 # Amarillo
        
        # Cerramos el rectángulo: P1 -> P2 -> P3 -> P4 -> P1
        points = [p1, p2, p3, p4, p1]
        for p in points:
            pt = Point()
            pt.x, pt.y, pt.z = float(p[0]), float(p[1]), 0.0
            marker.points.append(pt)
        
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ParkingDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()