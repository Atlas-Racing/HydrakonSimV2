#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import struct
import math
import numpy as np
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class ConeTrack:
    def __init__(self, x, y, z, cov, color_id, timestamp):
        self.x = x 
        self.y = y 
        self.z = z 
        self.cov = cov 
        self.color_probs = np.array([0.97, 0.01, 0.01, 0.01])
        if color_id != 0:
            self.update_color(color_id)
        
        self.id = -1 
        self.last_seen = timestamp
        self.obs_count = 1

    def update_color(self, color_id):
        if color_id == 0: return 
        likelihood = np.array([0.1, 0.1, 0.1, 0.1])
        if color_id == 1: likelihood = np.array([0.05, 0.9, 0.025, 0.025]) # Yellow
        elif color_id == 2: likelihood = np.array([0.05, 0.025, 0.9, 0.025]) # Blue
        elif color_id == 3: likelihood = np.array([0.05, 0.025, 0.025, 0.9]) # Orange
        self.color_probs *= likelihood
        self.color_probs /= np.sum(self.color_probs)

    def get_color(self):
        return np.argmax(self.color_probs)

    def update(self, z_x, z_y, z_z, R_cov, timestamp):
        H = np.eye(2)
        S = self.cov + R_cov
        try:
            K = self.cov @ np.linalg.inv(S)
        except:
            K = np.zeros((2,2))
        y = np.array([z_x - self.x, z_y - self.y])
        state_update = K @ y
        self.x += state_update[0]
        self.y += state_update[1]
        if z_z is not None:
             self.z = 0.9 * self.z + 0.1 * z_z
        self.cov = (np.eye(2) - K) @ self.cov
        self.last_seen = timestamp
        self.obs_count += 1

class ConeFusionNode(Node):
    def __init__(self):
        super().__init__('cone_fusion_node')
        
        # Filtering Parameters
        self.declare_parameter("lidar_min_radius", 2.5)
        self.declare_parameter("lidar_max_radius", 15.0) 
        self.declare_parameter("lidar_z_min", -3.0)
        self.declare_parameter("lidar_z_max", 1.0)
        
        self.min_r = self.get_parameter("lidar_min_radius").value
        self.max_r = self.get_parameter("lidar_max_radius").value
        self.z_min = self.get_parameter("lidar_z_min").value
        self.z_max = self.get_parameter("lidar_z_max").value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.tracks = []
        self.track_id_counter = 0
        
        self.lidar_sub = self.create_subscription(PointCloud2, '/carla/lidar', self.lidar_callback, 10)
        self.cam_sub = self.create_subscription(MarkerArray, '/camera/cone_markers', self.camera_callback, 10)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/fusion/cone_markers', 10)
        self.lidar_debug_pub = self.create_publisher(MarkerArray, '/lidar/debug_cones', 10)
        
        self.cam_x_offset = -6.5
        self.cam_z_offset = 2.5
        self.cam_pitch = math.radians(-15.0)

        self.get_logger().info("Cone Fusion Node (Camera-Dominant) Started")

    def get_transform(self, target_frame, source_frame):
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except Exception:
            return None

    def lidar_callback(self, msg):
        points = self.parse_pointcloud(msg)
        if not points: return
            
        filtered_points = []
        for p in points:
            x, y, z = p
            dist = math.hypot(x, y)
            
            # Vehicle Exclusion Box (Relative to LiDAR)
            # LiDAR at x=0.6 relative to car center.
            # Car extends roughly -2.0 to +2.0 in X, -1.2 to +1.2 in Y.
            # This box filters out chassis, wheels, and suspension.
            if (x > -3.0 and x < 1.5) and (y > -1.3 and y < 1.3):
                continue
            
            # Distance and Height Filters
            if dist < self.min_r or dist > self.max_r: continue
            if z < self.z_min or z > self.z_max: continue
            
            filtered_points.append([x, y, z])
            
        if not filtered_points: return
            
        clusters = self.cluster_points(filtered_points, tolerance=0.6, min_size=2)
        
        lidar_obs = []
        debug_markers = MarkerArray()
        
        # DELETEALL for debug
        m_del = Marker()
        m_del.action = 3 
        debug_markers.markers.append(m_del)

        transform = self.get_transform("odom", msg.header.frame_id)
        
        for i, cluster in enumerate(clusters):
            cx = np.mean([p[0] for p in cluster])
            cy = np.mean([p[1] for p in cluster])
            cz = np.mean([p[2] for p in cluster])
            
            m = Marker()
            m.header = msg.header
            m.ns = "lidar_raw"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.lifetime = Duration(seconds=0.1).to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(cx), float(cy), float(cz)
            m.scale.x, m.scale.y, m.scale.z = 0.4, 0.4, 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 1.0, 0.8
            debug_markers.markers.append(m)
            
            if transform:
                p_stamped = tf2_geometry_msgs.PointStamped()
                p_stamped.point.x, p_stamped.point.y, p_stamped.point.z = float(cx), float(cy), float(cz)
                try:
                    p_trans = tf2_geometry_msgs.do_transform_point(p_stamped, transform)
                    lidar_obs.append([p_trans.point.x, p_trans.point.y, p_trans.point.z])
                except: pass

        self.lidar_debug_pub.publish(debug_markers)
        
        if lidar_obs:
            # LiDAR Trust Reduced (25%) -> Higher Covariance (0.2)
            R_lidar = np.eye(2) * 0.2 
            self.update_tracks(lidar_obs, R_lidar, None)
            self.publish_tracks()

    def camera_callback(self, msg):
        transform_base_odom = self.get_transform("odom", "base_link")
        if not transform_base_odom: return

        cam_obs = []
        cp = math.cos(self.cam_pitch)
        sp = math.sin(self.cam_pitch)

        for marker in msg.markers:
            raw_x, raw_y, raw_z = marker.pose.position.x, marker.pose.position.y, marker.pose.position.z
            x_rot = raw_x * cp - raw_z * sp
            y_rot = raw_y
            z_rot = raw_x * sp + raw_z * cp
            x_base = x_rot + self.cam_x_offset
            y_base = y_rot
            z_base = z_rot + self.cam_z_offset
            
            p_stamped = tf2_geometry_msgs.PointStamped()
            p_stamped.point.x, p_stamped.point.y, p_stamped.point.z = float(x_base), float(y_base), float(z_base)
            
            try:
                p_trans = tf2_geometry_msgs.do_transform_point(p_stamped, transform_base_odom)
                r, g, b = marker.color.r, marker.color.g, marker.color.b
                color_id = 0
                if r > 0.9 and g > 0.9: color_id = 1 
                elif b > 0.9: color_id = 2 
                elif r > 0.9 and g > 0.4: color_id = 3 
                cam_obs.append(([p_trans.point.x, p_trans.point.y, p_trans.point.z], color_id))
            except: continue
        
        # Camera Trust Increased (75%) -> Lower Covariance (0.05)
        R_cam = np.eye(2) * 0.05 
        for pos, color_id in cam_obs:
            self.update_single_track(pos, R_cam, color_id)

    def update_tracks(self, observations, R, color_id_common=None):
        threshold = 1.0 
        for obs in observations:
            pos = obs
            best_dist = float('inf')
            best_idx = -1
            for i, track in enumerate(self.tracks):
                dist = math.hypot(track.x - pos[0], track.y - pos[1])
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            now = self.get_clock().now()
            
            if best_dist < threshold:
                self.tracks[best_idx].update(pos[0], pos[1], pos[2], R, now)
            else:
                new_track = ConeTrack(pos[0], pos[1], pos[2], R, 0, now) 
                new_track.id = self.track_id_counter
                self.track_id_counter += 1
                self.tracks.append(new_track)

    def update_single_track(self, pos, R, color_id):
        threshold = 2.5 
        best_dist = float('inf')
        best_idx = -1
        for i, track in enumerate(self.tracks):
            dist = math.hypot(track.x - pos[0], track.y - pos[1])
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        now = self.get_clock().now()
        if best_dist < threshold:
            self.tracks[best_idx].update(pos[0], pos[1], pos[2], R, now)
            self.tracks[best_idx].update_color(color_id)
        else:
            # Allow Camera to Create Tracks (Since we trust it more now)
            t = ConeTrack(pos[0], pos[1], pos[2], R, color_id, now)
            t.id = self.track_id_counter
            self.track_id_counter += 1
            self.tracks.append(t)

    def publish_tracks(self):
        now = self.get_clock().now()
        transform = self.get_transform("base_footprint", "odom")
        
        if not transform:
            return
        
        marker_array = MarkerArray()
        
        # DELETEALL
        m_del = Marker()
        m_del.action = 3 
        marker_array.markers.append(m_del)
        
        active_tracks = []
        for track in self.tracks:
            age = (now.nanoseconds - track.last_seen.nanoseconds) / 1e9
            if age < 0.5: 
                active_tracks.append(track)
            
            p_stamped = tf2_geometry_msgs.PointStamped()
            p_stamped.point.x, p_stamped.point.y, p_stamped.point.z = float(track.x), float(track.y), float(track.z)
            try:
                p_local = tf2_geometry_msgs.do_transform_point(p_stamped, transform)
                
                if p_local.point.x < -2.0 or p_local.point.x > 30.0: continue
                
                m = Marker()
                m.header.frame_id = "base_footprint"
                m.header.stamp = now.to_msg()
                m.ns = "fused_cones"
                m.id = track.id
                m.type = Marker.CYLINDER
                m.action = Marker.ADD
                m.lifetime = Duration(seconds=0.2).to_msg()
                
                m.pose.position.x = p_local.point.x
                m.pose.position.y = p_local.point.y
                m.pose.position.z = 0.0 
                
                m.scale.x, m.scale.y, m.scale.z = 0.3, 0.3, 0.6
                cid = track.get_color()
                if cid == 1: m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0 
                elif cid == 2: m.color.r, m.color.g, m.color.b = 0.0, 0.0, 1.0 
                elif cid == 3: m.color.r, m.color.g, m.color.b = 1.0, 0.5, 0.0 
                else: m.color.r, m.color.g, m.color.b = 1.0, 1.0, 1.0 
                m.color.a = 1.0
                marker_array.markers.append(m)
            except: continue
            
        self.tracks = active_tracks
        self.marker_pub.publish(marker_array)

    def parse_pointcloud(self, cloud_msg):
        points = []
        point_step = cloud_msg.point_step
        width = cloud_msg.width
        data = cloud_msg.data
        fmt = '<ffff' 
        for i in range(width):
            off = i * point_step
            try:
                x, y, z, _ = struct.unpack_from(fmt, data, off)
                points.append((x, y, z))
            except: break
        return points

    def cluster_points(self, points, tolerance, min_size):
        clusters = []
        processed = [False] * len(points)
        for i, p in enumerate(points):
            if processed[i]: continue
            cluster = [p]
            processed[i] = True
            queue = [p]
            while queue:
                curr = queue.pop(0)
                for j, neighbor in enumerate(points):
                    if not processed[j]:
                        dist = math.hypot(curr[0]-neighbor[0], curr[1]-neighbor[1])
                        if dist < tolerance:
                            processed[j] = True
                            cluster.append(neighbor)
                            queue.append(neighbor)
            if len(cluster) >= min_size:
                clusters.append(cluster)
        return clusters

def main(args=None):
    rclpy.init(args=args)
    node = ConeFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
