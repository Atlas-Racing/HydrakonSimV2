#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import torch
from ultralytics import YOLO

class ConeDetectorPT(Node):
    def __init__(self):
        super().__init__('cone_detector_pt_node')
        self.bridge = CvBridge()
        
        model_path = '/home/aditya/HydrakonSimV2/src/hydrakon_camera/hydrakon_camera/best.pt'
        
        self.model = YOLO(model_path)
        
        self.class_names = self.extract_class_names()
        
        self.class_colors = self.generate_class_colors()
        
        self.get_logger().info(f"PyTorch model loaded: {model_path}")
        self.get_logger().info(f"Detected classes: {self.class_names}")
        self.get_logger().info(f"Number of classes: {len(self.class_names)}")
        
        self.conf_threshold = 0.01 
        self.iou_threshold = 0.5
        
        self.image_sub = self.create_subscription(
            Image, '/camera/raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(
            Image, '/camera/cone_detections_image', 10)
        
        self.get_logger().info("Cone detector (PyTorch) node initialized")


    def extract_class_names(self):
        """Extract class names from the YOLO model"""
        try:
            if hasattr(self.model, 'names') and self.model.names:
                class_names = self.model.names.copy()
                
                if 4 in class_names and class_names[4] == 'unknown_cone':
                    if 3 in class_names and class_names[3] == 'large_orange_cone':
                        class_names[3], class_names[4] = class_names[4], class_names[3]
                        self.get_logger().info("Swapped unknown_cone (4) with large_orange_cone (3)")
                
                self.get_logger().info("Class names extracted from model.names")
                return class_names
            
            elif hasattr(self.model, 'model') and hasattr(self.model.model, 'names'):
                class_names = self.model.model.names
                self.get_logger().info("Class names extracted from model.model.names")
                return class_names
            
            elif hasattr(self.model, 'cfg') and 'names' in self.model.cfg:
                class_names = self.model.cfg['names']
                self.get_logger().info("Class names extracted from model.cfg")
                return class_names
            
            else:
                self.get_logger().warn("Could not automatically extract class names")
                num_classes = self.get_num_classes()
                class_names = {i: f"class_{i}" for i in range(num_classes)}
                self.get_logger().info(f"Using generic class names for {num_classes} classes")
                return class_names
                
        except Exception as e:
            self.get_logger().error(f"Error extracting class names: {str(e)}")
            return {0: "unknown", 1: "object"}
        

    def get_num_classes(self):
        """Get number of classes from the model"""
        try:
            if hasattr(self.model, 'model'):
                for module in self.model.model.modules():
                    if hasattr(module, 'nc'):
                        return module.nc
                    elif hasattr(module, 'anchors') and hasattr(module, 'no'):
                        return module.no - 5
            
            return 80
            
        except Exception as e:
            self.get_logger().error(f"Error getting number of classes: {str(e)}")
            return 5 
        

    def generate_class_colors(self):
        """Generate distinct colors for each class"""
        colors = {}
        num_classes = len(self.class_names)
        
        for i, class_name in enumerate(self.class_names.values() if isinstance(self.class_names, dict) else self.class_names):
            hue = int(180 * i / num_classes)
            color_hsv = np.uint8([[[hue, 255, 255]]])
            color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0][0]
            colors[class_name] = tuple(map(int, color_bgr))
        
        return colors
    

    def print_model_info(self):
        """Helper function to print detailed model information for debugging"""
        self.get_logger().info("=== MODEL INFORMATION ===")
        
        self.get_logger().info(f"Model attributes: {dir(self.model)}")
        
        if hasattr(self.model, 'names'):
            self.get_logger().info(f"model.names: {self.model.names}")
        
        if hasattr(self.model, 'model'):
            self.get_logger().info(f"model.model attributes: {dir(self.model.model)}")
            if hasattr(self.model.model, 'names'):
                self.get_logger().info(f"model.model.names: {self.model.model.names}")
        
        if hasattr(self.model, 'cfg'):
            self.get_logger().info(f"model.cfg: {self.model.cfg}")


    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame, conf=self.conf_threshold, iou=self.iou_threshold)
            
            annotated_frame = frame.copy()
            
            for result in results:
                boxes = result.boxes
                
                if boxes is not None and len(boxes) > 0:
                    xyxy = boxes.xyxy.cpu().numpy()
                    conf = boxes.conf.cpu().numpy()
                    cls = boxes.cls.cpu().numpy()
                    
                    for i in range(len(boxes)):
                        x1, y1, x2, y2 = xyxy[i].astype(int)
                        confidence = conf[i]
                        class_id = int(cls[i])
                        if isinstance(self.class_names, dict):
                            class_name = self.class_names.get(class_id, f"class_{class_id}")
                        else:
                            class_name = self.class_names[class_id] if class_id < len(self.class_names) else f"class_{class_id}"
                        self.get_logger().info(f"Raw detection: Class {class_id} ({class_name}), Confidence: {confidence:.3f}")
                        color = self.class_colors.get(class_name, (0, 255, 0))
                        label = f"{class_name} {confidence:.2f}"
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                        
                        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        cv2.rectangle(annotated_frame, (x1, y1 - label_size[1] - 10), 
                                    (x1 + label_size[0], y1), color, -1)
                        
                        cv2.putText(annotated_frame, label, (x1, y1 - 5),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        
                        self.get_logger().info(
                            f"Detected: {class_name} {confidence:.2f} at [{x1}, {y1}, {x2}, {y2}]"
                        )
            
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Image callback failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorPT()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()