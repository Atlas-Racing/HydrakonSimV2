#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import carla
import numpy as np
import threading
import sys

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        
        self.host = self.get_parameter("carla_host").get_parameter_value().string_value
        self.port = self.get_parameter("carla_port").get_parameter_value().integer_value
        
        self.publisher_ = self.create_publisher(PointCloud2, '/carla/lidar', 10)
        
        self.vehicle = None
        self.world = None
        self.lidar = None
        self.lidar_data = None
        self.data_lock = threading.Lock()
        
        self.get_logger().info("LiDAR Node Started.")
        
        self.timer = self.create_timer(0.05, self.process_and_publish)
        
        self.find_vehicle_timer = self.create_timer(1.0, self.find_and_setup_vehicle)
        
        self.connect_carla()
    
    def connect_carla(self):
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(5.0)
            self.world = client.get_world()
        except Exception as e:
            self.get_logger().error(f"Error connecting to CARLA: {str(e)}")

    def find_and_setup_vehicle(self):
        if self.vehicle:
            return

        actors = self.world.get_actors().filter('vehicle.*')
        if not actors:
            self.get_logger().warn("Waiting for vehicle...")
            return
            
        self.vehicle = actors[0]
        self.get_logger().info(f"Found vehicle: {self.vehicle.type_id}. Spawning LiDAR...")
        self.setup_lidar()
        
        self.find_vehicle_timer.cancel()
    
    def setup_lidar(self):
        try:
            blueprint_library = self.world.get_blueprint_library()
            lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
            
            lidar_bp.set_attribute('channels', '64')
            lidar_bp.set_attribute('points_per_second', '500000')
            lidar_bp.set_attribute('rotation_frequency', '20')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('upper_fov', '15')
            lidar_bp.set_attribute('lower_fov', '-25')
            
            lidar_transform = carla.Transform(carla.Location(x=1.5, z=2.2))
            self.lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
            
            self.lidar.listen(lambda data: self.lidar_callback(data))
            self.get_logger().info("LiDAR sensor attached and streaming.")
            
        except Exception as e:
            self.get_logger().error(f"Error setting up LiDAR: {str(e)}")

    def lidar_callback(self, data):
        with self.data_lock:
            self.lidar_data = data

    def process_and_publish(self):
        with self.data_lock:
            if self.lidar_data is None:
                return
            data = self.lidar_data
            self.lidar_data = None 
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "lidar_link" 
        
        raw_array = np.frombuffer(data.raw_data, dtype=np.float32)
        points = raw_array.reshape(-1, 4).copy()
        
        points[:, 1] = -points[:, 1]
        
        msg = self.create_pointcloud2(header, points)
        self.publisher_.publish(msg)

    def create_pointcloud2(self, header, points):
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = points.shape[0]
        msg.is_dense = False
        msg.is_bigendian = False
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.data = points.tobytes()
        
        return msg

    def destroy(self):
        if self.lidar:
            self.lidar.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
