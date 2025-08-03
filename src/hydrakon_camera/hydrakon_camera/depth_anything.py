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
        self.bridge = CvBridge()

        self.rgb_subscriber = self.create_subscription(
            Image,
            '/camera',
            self.rgb_callback,
            10
        )

        self.depth_publisher = self.create_publisher(
            Image,
            '/camera/depthanything',
            10
        )

        self.colored_depth_publisher = self.create_publisher(
            Image,
            '/camera/depthanything_colored',
            10
        )

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

                try:
                    if hasattr(torch, 'compile') and torch.__version__ >= '2.0':
                        self.get_logger().info("Applying torch.compile() optimization...")
                        self.model = torch.compile(self.model, mode='reduce-overhead')
                        self.get_logger().info("torch.compile() optimization applied successfully")
                    else:
                        self.get_logger().info("torch.compile() not available (requires PyTorch 2.0+)")
                except Exception as e:
                    self.get_logger().warn(f"torch.compile() failed, continuing without optimization: {str(e)}")
                
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

            with torch.no_grad():
                depth_map = self.model.infer_image(cv_image)
            
            depth_normalized = (depth_map - depth_map.min()) / (depth_map.max() - depth_map.min())

            depth_16bit = (depth_normalized * 65535).astype(np.uint16)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_16bit, encoding="16UC1")
            depth_msg.header = msg.header
            depth_msg.header.frame_id = "camera_depth_anything_frame"
            self.depth_publisher.publish(depth_msg)
            
            # Change as required:
            # cv2.COLORMAP_AUTUMN - Red to yellow gradient
            # cv2.COLORMAP_BONE - Grayscale with blue tint
            # cv2.COLORMAP_JET - Blue → Cyan → Yellow → Red (most common for depth)
            # cv2.COLORMAP_WINTER - Blue to green gradient
            # cv2.COLORMAP_RAINBOW - Full spectrum colors
            # cv2.COLORMAP_OCEAN - Dark blue to cyan
            # cv2.COLORMAP_SUMMER - Green to yellow gradient
            # cv2.COLORMAP_SPRING - Magenta to yellow gradient
            # cv2.COLORMAP_COOL - Cyan to magenta gradient
            # cv2.COLORMAP_HSV - HSV color wheel
            # cv2.COLORMAP_PINK - Pink gradient
            # cv2.COLORMAP_HOT - Black → Red → Yellow → White
            # cv2.COLORMAP_CIVIDIS - Blue → Yellow (colorblind-friendly) 
            # cv2.COLORMAP_VIRIDIS - Purple → Blue → Green → Yellow
            # cv2.COLORMAP_PLASMA - Purple → Pink → Yellow
            # cv2.COLORMAP_MAGMA - Black → Purple → Pink → Yellow
            # cv2.COLORMAP_PARULA - MATLAB's default colormap

            # Lowkey fire color maps:
            # cv2.COLORMAP_TURBO - Google's Turbo colormap (improved JET)
            # cv2.COLORMAP_TWILIGHT - Pink → White → Cyan
            # cv2.COLORMAP_TWILIGHT_SHIFTED - Shifted version of twilight
            # cv2.COLORMAP_INFERNO - Black → Purple → Red → Yellow

            # depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_HSV)
            # depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            depth_colored = cv2.applyColorMap((depth_normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            colored_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding="bgr8")
            colored_msg.header = msg.header
            colored_msg.header.frame_id = "camera_depth_anything_colored_frame"
            self.colored_depth_publisher.publish(colored_msg)
                
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