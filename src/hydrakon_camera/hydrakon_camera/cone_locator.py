#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import math

class ConeLocatorNode(Node):
    def __init__(self):
        super().__init__('cone_locator_node')
        
        self.bridge = CvBridge()
        
        self.fx = 400.0
        self.fy = 400.0
        self.cx = 400.0
        self.cy = 300.0
        
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth')
        self.detections_sub = message_filters.Subscriber(self, Detection2DArray, '/camera/cone_detections')
        
        self.marker_pub = self.create_publisher(MarkerArray, '/camera/cone_markers', 10)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.detections_sub], 
            queue_size=10, 
            slop=0.1 
        )
        self.ts.registerCallback(self.callback)
        
        self.get_logger().info("Cone Locator Node initialized. Waiting for synced messages...")

    def callback(self, depth_msg, detections_msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            marker_array = MarkerArray()
            
            # Map class IDs to colors (R, G, B)
            # 0: unknown, 1: yellow, 2: blue, 3: orange, 4: large_orange
            # YOLO.py output: {0: 'unknown_cone', 1: 'yellow_cone', 2: 'blue_cone', 3: 'orange_cone', 4: 'large_orange_cone'}
            colors = {
                0: (1.0, 1.0, 1.0), # Unknown (White)
                1: (1.0, 1.0, 0.0), # Yellow
                2: (0.0, 0.0, 1.0), # Blue
                3: (1.0, 0.5, 0.0), # Orange
                4: (1.0, 0.5, 0.0)  # Large Orange
            }
            
            for i, detection in enumerate(detections_msg.detections):
                u = int(detection.bbox.center.position.x)
                v = int(detection.bbox.center.position.y)
                w = int(detection.bbox.size_x)
                h = int(detection.bbox.size_y)
                
                if u < 0 or u >= depth_image.shape[1] or v < 0 or v >= depth_image.shape[0]:
                    continue
                
                # Median depth calculation
                x1 = max(0, int(u - w/2))
                y1 = max(0, int(v - h/2))
                x2 = min(depth_image.shape[1], int(u + w/2))
                y2 = min(depth_image.shape[0], int(v + h/2))
                
                roi = depth_image[y1:y2, x1:x2]
                if roi.size == 0: continue
                
                valid_depths = roi[np.isfinite(roi) & (roi > 0)]
                if valid_depths.size == 0: continue
                    
                depth = np.median(valid_depths)
                
                # 3D Position (Camera Frame)
                z_cam = float(depth)
                x_cam = float((u - self.cx) * z_cam / self.fx)
                y_cam = float((v - self.cy) * z_cam / self.fy)
                
                class_id = int(detection.results[0].hypothesis.class_id)
                
                # Create Marker
                marker = Marker()
                marker.header.frame_id = "cone_frame"
                marker.header.stamp = depth_msg.header.stamp
                marker.ns = "cones"
                marker.id = i
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                
                # Coordinate Transform: Camera (Z=Fwd, X=Right, Y=Down) -> Vehicle (X=Fwd, Y=Left, Z=Up)
                marker.pose.position.x = z_cam
                marker.pose.position.y = -x_cam
                marker.pose.position.z = -y_cam
                
                # Orientation (Default upright cylinder aligns with Z, which is now Up)
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                
                # Scale
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.5 if class_id != 4 else 0.8 # Taller for large cones
                
                # Color
                rgb = colors.get(class_id, (1.0, 1.0, 1.0))
                marker.color.r = rgb[0]
                marker.color.g = rgb[1]
                marker.color.b = rgb[2]
                marker.color.a = 1.0
                
                marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
                
                marker_array.markers.append(marker)
                
                # self.get_logger().info(f"Cone {class_id} at ({x:.2f}, {y:.2f}, {z:.2f})")

            self.marker_pub.publish(marker_array)
                
        except Exception as e:
            self.get_logger().error(f"Error in callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ConeLocatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
