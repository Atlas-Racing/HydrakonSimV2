#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import onnxruntime as ort

class ConeDetectorONNX(Node):
    def __init__(self):
        super().__init__('cone_detector_onnx_node')
        self.bridge = CvBridge()
        
        # Update this path to your ONNX model
        model_path = '/home/aditya/HydrakonSimV2/src/hydrakon_camera/hydrakon_camera/best.onnx'
        
        # Initialize ONNX Runtime session
        self.session = ort.InferenceSession(model_path)
        
        # Get model input/output details
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.output_names = [output.name for output in self.session.get_outputs()]
        
        self.get_logger().info(f"ONNX model loaded: {model_path}")
        self.get_logger().info(f"Input shape: {self.input_shape}")
        self.get_logger().info(f"Output names: {self.output_names}")
        
        # Model parameters
        self.img_size = 640  # Standard YOLOv8 input size
        self.conf_threshold = 0.01  # Increased threshold to filter noise
        self.iou_threshold = 0.5
        
        # Get class names from original PT model
        self.class_names = {
            0: "unknown_cone",
            1: "yellow_cone",
            2: "blue_cone",
            3: "orange_cone",
            4: "large_orange_cone"
        }
        
        self.class_colors = {
            "unknown_cone": (128, 128, 128),     # Gray
            "yellow_cone": (0, 255, 255),        # Yellow
            "blue_cone": (255, 0, 0),            # Blue
            "orange_cone": (0, 140, 255),        # Orange
            "large_orange_cone": (255, 165, 0)   # Light orange
        }
        
        # ROS2 subscriptions and publishers
        self.image_sub = self.create_subscription(
            Image, '/camera/raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(
            Image, '/camera/cone_detections_image', 10)
        
        self.get_logger().info("Cone detector (ONNX) node initialized")

    def preprocess_image(self, image):
        """Preprocess image for YOLO inference"""
        # Resize image to model input size
        img = cv2.resize(image, (self.img_size, self.img_size))
        
        # Convert BGR to RGB
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        # Normalize to [0, 1] and convert to float32
        img = img.astype(np.float32) / 255.0
        
        # Add batch dimension and transpose to NCHW format
        img = np.transpose(img, (2, 0, 1))  # HWC to CHW
        img = np.expand_dims(img, axis=0)   # Add batch dimension
        
        return img

    def postprocess_detections(self, outputs, orig_shape):
        """Post-process YOLO outputs to get bounding boxes"""
        # YOLOv8 ONNX output format: [4 + num_classes, num_detections]
        # Shape is (9, 8400) = (4 bbox + 5 classes, 8400 detections)
        predictions = outputs[0]  # Shape: [9, 8400]
        
        # Debug: Print output shape
        self.get_logger().info(f"ONNX output shape: {predictions.shape}")
        
        # Transpose to [num_detections, 4 + num_classes]
        predictions = predictions.T  # Now shape: [8400, 9]
        
        boxes = []
        scores = []
        class_ids = []
        
        orig_h, orig_w = orig_shape[:2]
        scale_x = orig_w / self.img_size
        scale_y = orig_h / self.img_size
        
        # Process each detection
        for detection in predictions:
            # Extract coordinates (center format, normalized to input size)
            x_center, y_center, width, height = detection[:4]
            
            # Get class scores (everything after first 4 values)
            class_scores = detection[4:]
            
            # Find class with highest confidence
            class_id = np.argmax(class_scores)
            confidence = class_scores[class_id]
            
            # Filter by confidence threshold
            if confidence > self.conf_threshold:
                # Convert center format to corner format and scale to original image
                x1 = max(0, (x_center - width / 2) * scale_x)
                y1 = max(0, (y_center - height / 2) * scale_y)
                x2 = min(orig_w, (x_center + width / 2) * scale_x)
                y2 = min(orig_h, (y_center + height / 2) * scale_y)
                
                # Validate bounding box
                if x2 > x1 and y2 > y1 and (x2 - x1) > 5 and (y2 - y1) > 5:
                    boxes.append([x1, y1, x2, y2])
                    scores.append(float(confidence))
                    class_ids.append(int(class_id))
        
        # Apply Non-Maximum Suppression
        if len(boxes) > 0:
            boxes = np.array(boxes)
            scores = np.array(scores)
            class_ids = np.array(class_ids)
            
            # OpenCV NMS
            indices = cv2.dnn.NMSBoxes(
                boxes.tolist(), scores.tolist(), 
                self.conf_threshold, self.iou_threshold
            )
            
            if len(indices) > 0:
                indices = indices.flatten()
                return boxes[indices], scores[indices], class_ids[indices]
        
        return [], [], []

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            annotated_frame = frame.copy()
            orig_shape = frame.shape
            
            # Preprocess image for inference
            input_image = self.preprocess_image(frame)
            
            # Run ONNX inference
            outputs = self.session.run(self.output_names, {self.input_name: input_image})
            
            # Post-process detections
            boxes, scores, class_ids = self.postprocess_detections(outputs[0], orig_shape)
            
            # Draw detections
            for box, score, class_id in zip(boxes, scores, class_ids):
                x1, y1, x2, y2 = box.astype(int)
                
                class_name = self.class_names.get(class_id, f"class_{class_id}")
                color = self.class_colors.get(class_name, (0, 255, 0))
                label = f"{class_name} {score:.2f}"

                cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(annotated_frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

                self.get_logger().info(
                    f"Detected: {class_name} {score:.2f} at [{x1}, {y1}, {x2}, {y2}]"
                )
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            annotated_msg.header = msg.header
            self.image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Image callback failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectorONNX()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()