#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os


class DepthAnythingProcessor(Node):
    def __init__(self):
        super().__init__('depth_anything_processor')
        
        # ROS setup
        self.bridge = CvBridge()
        
        # Subscribe to RGB camera feed
        self.rgb_subscriber = self.create_subscription(
            Image,
            '/camera',
            self.rgb_callback,
            10
        )
        
        # Publisher for depth estimation
        self.depth_publisher = self.create_publisher(
            Image,
            '/camera/depthanything',
            10
        )
        
        # Initialize Depth Anything V2 model
        self.setup_depth_model()
        
        self.get_logger().info("Depth Anything V2 processor initialized")

    
    def setup_depth_model(self):
        """Initialize Depth Anything V2 model"""
        try:
            self.device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
            self.get_logger().info(f"Using device: {self.device}")

            try:
                from depth_anything_v2.dpt import DepthAnythingV2
                model_configs = {
                    'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
                    'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
                    'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
                    'vitg': {'encoder': 'vitg', 'features': 384, 'out_channels': [1536, 1536, 1536, 1536]}
                }

                encoder = 'vits'
                
                self.model = DepthAnythingV2(**model_configs[encoder])
                checkpoint_path = f'checkpoints/depth_anything_v2_{encoder}.pth'
                
                if os.path.exists(checkpoint_path):
                    self.model.load_state_dict(torch.load(checkpoint_path, map_location='cpu'))
                    self.get_logger().info(f"Loaded Depth Anything V2 weights from {checkpoint_path}")
                else:
                    self.get_logger().error(f"Model weights not found at {checkpoint_path}")
                    self.get_logger().error("Please download the weights and put them in the 'checkpoints' directory")
                    self.get_logger().error("See: https://github.com/DepthAnything/Depth-Anything-V2")
                    return
                
                self.model = self.model.to(self.device).eval()
                
                self.get_logger().info(f"Depth Anything V2 model ({encoder}) loaded successfully")
                
            except ImportError:
                self.get_logger().error("Depth Anything V2 not installed. Please run:")
                self.get_logger().error("git clone https://github.com/DepthAnything/Depth-Anything-V2")
                self.get_logger().error("cd Depth-Anything-V2")
                self.get_logger().error("pip install -r requirements.txt")
                self.model = None
                
        except Exception as e:
            self.get_logger().error(f"Error setting up Depth Anything V2: {str(e)}")
            self.model = None

    
    def rgb_callback(self, msg):
        """Process RGB image and estimate depth"""
        if self.model is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            depth_map = self.model.infer_image(cv_image)
            
            depth_normalized = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())
            
            depth_16bit = (depth_normalized * 65535).astype(np.uint16)
            
            depth_msg = self.bridge.cv2_to_imgmsg(depth_16bit, encoding="16UC1")
            depth_msg.header = msg.header  # Copy original header
            depth_msg.header.frame_id = "camera_depth_anything_frame"
            
            # Publish depth estimation
            self.depth_publisher.publish(depth_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing RGB image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DepthAnythingProcessor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in main: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()