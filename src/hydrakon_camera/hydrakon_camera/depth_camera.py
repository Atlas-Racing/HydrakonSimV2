#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import carla
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class DepthCameraSpawnerNode(Node):
    def __init__(self):
        super().__init__('carla_camera_spawner')
        
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("camera_width", 800)
        self.declare_parameter("camera_height", 600)
        self.declare_parameter("camera_fov", 90.0)
        
        self.host = self.get_parameter("carla_host").get_parameter_value().string_value
        self.port = self.get_parameter("carla_port").get_parameter_value().integer_value
        self.width = self.get_parameter("camera_width").get_parameter_value().integer_value
        self.height = self.get_parameter("camera_height").get_parameter_value().integer_value
        self.fov = self.get_parameter("camera_fov").get_parameter_value().double_value
        
        self.camera = None
        self.world = None
        self.vehicle = None
        
        self.depth_publisher = self.create_publisher(Image, '/camera/depth', 10)
        self.bridge = CvBridge()
        
        self.setup()
        
        self.get_logger().info("Camera spawner node initialized.")
    
    def setup(self):
        """Connect to CARLA, find a vehicle, and attach a depth camera."""
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(5.0)
            self.get_logger().info(f"Connecting to Carla at {self.host}:{self.port}")
            
            self.world = client.get_world()
            self.vehicle = self.find_vehicle()
            
            if self.vehicle:
                self.get_logger().info(f"Found vehicle with ID: {self.vehicle.id}")
                self.camera = self.spawn_depth_camera()
                
                if self.camera:
                    self.get_logger().info(f"Depth camera attached to vehicle with ID: {self.camera.id}")
                else:
                    self.get_logger().error("Failed to spawn depth camera.")
            else:
                self.get_logger().error("No vehicle found in the world. Spawn a vehicle first.")
                
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {str(e)}")

    
    def find_vehicle(self):
        """Find an existing vehicle in the CARLA world."""
        try:
            actors = self.world.get_actors()
            vehicles = actors.filter('vehicle.*')
            
            if len(vehicles) > 0:
                vehicle = vehicles[0]
                self.get_logger().info(f"Found vehicle: {vehicle.type_id} at location {vehicle.get_location()}")
                return vehicle
            else:
                self.get_logger().warning("No vehicles found in the world")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error finding vehicle: {str(e)}")
            return None
        
    
    def spawn_depth_camera(self):
        """Spawn a depth camera and attach it to the vehicle."""
        try:
            blueprint_library = self.world.get_blueprint_library()
            camera_bp = blueprint_library.find('sensor.camera.depth')
            
            if not camera_bp:
                self.get_logger().error("Depth camera blueprint not found")
                return None
            
            camera_bp.set_attribute('image_size_x', str(self.width))
            camera_bp.set_attribute('image_size_y', str(self.height))
            camera_bp.set_attribute('fov', str(self.fov))
            
            self.get_logger().info(f"Camera settings: {self.width}x{self.height}, FOV: {self.fov}")
            
            camera_transform = carla.Transform(
                carla.Location(x=-1.0, y=0.0, z=0.7),
                carla.Rotation(pitch=-5.0, yaw=0.0, roll=0.0)
            )
            
            camera = self.world.spawn_actor(
                camera_bp,
                camera_transform,
                attach_to=self.vehicle
            )
            
            if camera:
                self.get_logger().info("Depth camera successfully attached to vehicle")
                
                camera.listen(self.camera_callback)
                
                return camera
            else:
                self.get_logger().error("Failed to spawn depth camera")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Error spawning depth camera: {str(e)}")
            return None
        
    
    def camera_callback(self, image):
        """Callback function to process depth camera data and publish to ROS topic."""
        try:
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]  # Convert BGR to RGB
            
            R = array[:, :, 0].astype(np.float32)
            G = array[:, :, 1].astype(np.float32)
            B = array[:, :, 2].astype(np.float32)
            
            depth_normalized = (R + G * 256.0 + B * 256.0 * 256.0) / (256.0 * 256.0 * 256.0 - 1.0)
            depth_array = depth_normalized * 1000.0
            
            ros_image = self.bridge.cv2_to_imgmsg(depth_array, encoding="32FC1")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_depth_frame"
            
            self.depth_publisher.publish(ros_image)
                
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {str(e)}")

    
    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        self.get_logger().info("Shutting down camera spawner node...")
        
        if self.camera and self.camera.is_alive:
            self.camera.destroy()
            self.get_logger().info("Depth camera destroyed")
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DepthCameraSpawnerNode()
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