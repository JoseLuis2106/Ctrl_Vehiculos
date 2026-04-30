#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_geometry_msgs import PoseStamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
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
        
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_lidar)
        self.marker_pub = self.create_publisher(Marker, '/parking_slot_marker', qos_viz)
        self.cluster_pub = self.create_publisher(MarkerArray, '/viz_clusters', qos_viz)
        self.goal_pub = self.create_publisher(PoseStamped, '/parking_goal', qos_viz)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Configuración de TF2 para transformar coordenadas
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.min_slot_width = 3.8   
        self.max_slot_width = 7.5   
        
        self.state = "SEARCHING"
        self.path_requested = False

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.4 
        self.cmd_pub.publish(msg)

    def stop_car(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.cmd_pub.publish(msg)
        self.state = "STOPPED"
        self.get_logger().info("¡HUECO ALCANZADO! Coche detenido.")

    def send_path_request(self, goal_pose_odom):
        """Envía la meta transformada en ODOM a la acción de Nav2"""
        if self.path_requested:
            return

        # Corregido: nombre del cliente coincidente con __init__
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Acción NavigateToPose no disponible')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose_odom # <-- IMPORTANTE: Usar la pose en ODOM
        
        self.get_logger().info('Enviando meta definitiva a Nav2...')
        self.path_requested = True
        
        # Conectamos el callback para recibir la respuesta del servidor
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def publish_goal(self, p1, p2, center):
        # 1. Calculamos la pose en local (base_footprint) como ya hacías
        goal_base = PoseStamped()
        goal_base.header.frame_id = "base_footprint" 
        goal_base.header.stamp = rclpy.time.Time().to_msg()
        
        yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        inward_angle = yaw - (math.pi / 2.0)
        dist_offset = 1.2 # Un poco más de margen para que Nav2 maniobre

        goal_base.pose.position.x = center[0] + dist_offset * math.cos(inward_angle)
        goal_base.pose.position.y = center[1] + dist_offset * math.sin(inward_angle)
        
        goal_base.pose.orientation.z = math.sin(yaw / 2.0)
        goal_base.pose.orientation.w = math.cos(yaw / 2.0)

        # 2. Transformamos a ODOM antes de enviar a Nav2
        try:
            goal_odom = self.tf_buffer.transform(goal_base, "odom", timeout=rclpy.duration.Duration(seconds=0.1))
            
            # 3. PUBLICAMOS Y ENVIAMOS ACCIÓN
            self.goal_pub.publish(goal_odom)
            self.send_path_request(goal_odom)
            
        except Exception as e:
            self.get_logger().error(f"Fallo en la transformación: {str(e)}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Planificación rechazada por Nav2')
            self.path_requested = False
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("¡Meta alcanzada con éxito!")
        else:
            self.get_logger().warn(f"La meta falló con estado: {status}")

    def scan_callback(self, msg):
        if self.state != "STOPPED":
            self.move_forward()        
    
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        mask = (ranges > 0.1) & (ranges < 10.0)
        if not np.any(mask): return
        
        points = np.column_stack((ranges[mask] * np.cos(angles[mask]), ranges[mask] * np.sin(angles[mask])))

        # Clustering
        dist_to_sensor = np.linalg.norm(points, axis=1)
        depth_diff = np.abs(dist_to_sensor[1:] - dist_to_sensor[:-1])
        spatial_diff = np.linalg.norm(points[1:] - points[:-1], axis=1)
        indices = np.where((spatial_diff > 0.3) | (depth_diff > 0.4))[0] + 1
        clusters = np.split(points, indices)
        clusters = [c for c in clusters if len(c) > 3]
        
        self.viz_clusters(clusters)

        if len(clusters) < 3: return

        for i in range(len(clusters) - 2):
            c_left, c_mid, c_right = clusters[i], clusters[i+1], clusters[i+2]
            d_left = np.min(np.linalg.norm(c_left, axis=1))
            d_mid = np.min(np.linalg.norm(c_mid, axis=1))
            d_right = np.min(np.linalg.norm(c_right, axis=1))

            if d_mid > (d_left + 0.5) and d_mid > (d_right + 0.5):
                p1 = c_left[np.argmin(np.linalg.norm(c_left, axis=1))]
                p2 = c_right[np.argmin(np.linalg.norm(c_right, axis=1))]
                width = np.linalg.norm(p2 - p1)

                if self.min_slot_width < width < self.max_slot_width and self.state != "STOPPED":
                    center = (p1 + p2) / 2
                    self.publish_marker(center, width)

                    if center[0] < -0.5: 
                        self.stop_car()
                        self.publish_goal(p1, p2, center)
                    else:
                        self.move_forward()


    def viz_clusters(self, clusters):
        marker_array = MarkerArray()
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = "base_footprint" 
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

    def publish_marker(self, pos, width):
        marker = Marker()
        marker.header.frame_id = "base_footprint" 
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = f"PARKING: {width:.2f}m"
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = pos[0], pos[1], 1.2
        marker.scale.z = 0.4
        marker.color.a, marker.color.g = 1.0, 1.0
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