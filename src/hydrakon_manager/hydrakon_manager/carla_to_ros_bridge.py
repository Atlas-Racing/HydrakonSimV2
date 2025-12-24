#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import carla
import math
import time

class CarlaToRosBridge(Node):
    def __init__(self):
        super().__init__('carla_to_ros_bridge')

        # Parameters
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("role_name", "ego_vehicle")

        self.host = self.get_parameter("carla_host").value
        self.port = self.get_parameter("carla_port").value
        self.role_name = self.get_parameter("role_name").value

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.client = None
        self.world = None
        self.vehicle = None
        
        self.wheel_rotation_angle = 0.0
        self.last_time = time.time()

        self.connect_carla()
        
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("Carla-to-ROS Bridge Node Started.")

    def connect_carla(self):
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(5.0)
            self.world = self.client.get_world()
            self.find_vehicle()
        except Exception as e:
            self.get_logger().error(f"Carla connection failed: {e}")

    def find_vehicle(self):
        actors = self.world.get_actors()
        vehicles = actors.filter('vehicle.*')
        
        if len(vehicles) > 0:
            self.vehicle = vehicles[0]
            self.get_logger().info(f"Attached to vehicle: {self.vehicle.type_id} (ID: {self.vehicle.id})")
        else:
            self.get_logger().warn("No vehicle found yet. Retrying...")
            self.vehicle = None

    def timer_callback(self):
        if self.vehicle is None or not self.vehicle.is_alive:
            self.find_vehicle()
            return

        try:
            control = self.vehicle.get_control()
            velocity = self.vehicle.get_velocity()
            
            speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            transform = self.vehicle.get_transform()
            fwd = transform.get_forward_vector()
            dot = (velocity.x * fwd.x) + (velocity.y * fwd.y) + (velocity.z * fwd.z)
            if dot < 0:
                speed = -speed

        except RuntimeError:
            self.vehicle = None
            return

        max_steer_rad = 0.6
        steering_angle = -1.0 * control.steer * max_steer_rad

        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        wheel_radius = 0.3
        angular_vel = speed / wheel_radius
        self.wheel_rotation_angle += angular_vel * dt
        
        self.wheel_rotation_angle %= (2 * math.pi)

        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.name = [
            'front_left_steering_joint',
            'front_right_steering_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        msg.position = [
            steering_angle,
            steering_angle,
            self.wheel_rotation_angle,
            self.wheel_rotation_angle,
            self.wheel_rotation_angle,
            self.wheel_rotation_angle
        ]
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CarlaToRosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
