#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import carla
import time


class VehicleSpawnerNode(Node):
    def __init__(self):
        super().__init__('carla_vehicle_spawner')
        
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        
        self.host = self.get_parameter("carla_host").get_parameter_value().string_value
        self.port = self.get_parameter("carla_port").get_parameter_value().integer_value
        
        self.vehicle = None
        self.world = None
        
        self.setup()
        
        self.get_logger().info("Vehicle spawner node initialized.")
    
    def setup(self):
        """Connect to CARLA and spawn a vehicle."""
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(5.0)
            self.get_logger().info(f"Connecting to Carla at {self.host}:{self.port}")
            
            self.world = client.get_world()
            self.vehicle = self.spawn_vehicle()
            
            if self.vehicle:
                self.get_logger().info(f"Vehicle successfully spawned with ID: {self.vehicle.id}")
            else:
                self.get_logger().error("Vehicle could not be spawned.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {str(e)}")
    
    def spawn_vehicle(self):
        """Spawn a vehicle in the CARLA world."""
        try:
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = blueprint_library.filter('vehicle.*')[2]
            
            if not vehicle_bp:
                self.get_logger().error("No vehicle blueprints found")
                return None
                
            self.get_logger().info(f"Using vehicle blueprint: {vehicle_bp.id}")
            
            # Trackdrive spawn point
            spawn_transform = carla.Transform(
                carla.Location(x=-37.0, y=-1.0, z=5.0),
                carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            )

            # Autocross spawn point
            # spawn_transform = carla.Transform(
            #     carla.Location(x=95.0, y=-2.0, z=5.0),
            #     carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            # )

            # Skidpad spawn point
            # spawn_transform = carla.Transform(
            #     carla.Location(x=-90.0, y=-36.0, z=5.0),
            #     carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            # )

            # Acceleration spawn point
            # spawn_transform = carla.Transform(
            #     carla.Location(x=-90.0, y=-92.0, z=5.0),
            #     carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
            # )
            
            vehicle = self.world.try_spawn_actor(vehicle_bp, spawn_transform)
            
            if vehicle:
                self.get_logger().info(f"Vehicle spawned at {spawn_transform.location}")
                time.sleep(2.0)
                
                if not vehicle.is_alive:
                    self.get_logger().error("Vehicle failed to spawn or is not alive")
                    return None
                return vehicle
            else:
                self.get_logger().error("Spawn location occupied. Vehicle not spawned.")
                return None
        except Exception as e:
            self.get_logger().error(f"Error spawning vehicle: {str(e)}")
            return None
                
    def destroy_node(self):
        """Clean up resources when the node is destroyed."""
        self.get_logger().info("Shutting down vehicle spawner node...")
            
        if self.vehicle and self.vehicle.is_alive:
            self.vehicle.destroy()
            self.get_logger().info("Vehicle destroyed")
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = VehicleSpawnerNode()
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