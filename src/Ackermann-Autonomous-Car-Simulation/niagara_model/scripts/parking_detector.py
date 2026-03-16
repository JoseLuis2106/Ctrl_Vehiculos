#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import colorsys
import math

# Necesitas tener instalado: sudo apt install ros-humble-tf2-geometry-msgs
import tf2_ros
import tf2_geometry_msgs

class ParkingDetector(Node):
    def __init__(self):
        super().__init__('parking_detector')

        qos_lidar = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        qos_viz = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_lidar)
        self.marker_pub = self.create_publisher(Marker, '/parking_slot_marker', qos_viz)
        self.cluster_pub = self.create_publisher(MarkerArray, '/viz_clusters', qos_viz)
        self.goal_pub = self.create_publisher(PoseStamped, '/parking_goal', qos_viz)

        # Configuración de TF2 para transformar coordenadas
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.min_slot_width = 3.8   
        self.max_slot_width = 7.5   

    def scan_callback(self, msg):
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
                # Usamos argmin para p1 y p2 (tu mejora)
                p1 = c_left[np.argmin(np.linalg.norm(c_left, axis=1))]
                p2 = c_right[np.argmin(np.linalg.norm(c_right, axis=1))]
                width = np.linalg.norm(p2 - p1)

                if self.min_slot_width < width < self.max_slot_width:
                    center = (p1 + p2) / 2
                    self.publish_marker(center, width)
                    self.publish_goal(p1, p2, center)

    def publish_goal(self, p1, p2, center):
        goal_base = PoseStamped()
        # Cambiamos base_link por base_footprint
        goal_base.header.frame_id = "base_footprint" 
        goal_base.header.stamp = self.get_clock().now().to_msg()
        
        yaw = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        inward_angle = yaw - (math.pi / 2.0)
        dist_offset = 1.0

        goal_base.pose.position.x = center[0] + dist_offset * math.cos(inward_angle)
        goal_base.pose.position.y = center[1] + dist_offset * math.sin(inward_angle)
        
        goal_base.pose.orientation.z = math.sin(yaw / 2.0)
        goal_base.pose.orientation.w = math.cos(yaw / 2.0)

        try:
            latest_time = rclpy.time.Time() 

            if self.tf_buffer.can_transform("odom", "base_footprint", latest_time, rclpy.duration.Duration(seconds=0.1)):
                
                transform = self.tf_buffer.lookup_transform(
                    "odom", 
                    "base_footprint", 
                    latest_time
                )
                
                goal_odom = tf2_geometry_msgs.do_pose_stamped_transform(goal_base, transform)
                
                self.goal_pub.publish(goal_odom)
            
        except Exception as e:
            self.get_logger().debug(f"Esperando sincronización de frames... {str(e)}")
            self.goal_pub.publish(goal_base)

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
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': 
    main()