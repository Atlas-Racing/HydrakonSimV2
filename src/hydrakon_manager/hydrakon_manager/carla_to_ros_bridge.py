#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
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

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.client = None
        self.world = None
        self.vehicle = None
        
        self.wheel_rotation_angle = 0.0
        self.last_time = time.time()
        
        # Initial position offset (to make spawn point (0,0,0) if needed)
        # Not used for now, as we assume odom is global
        # self.initial_transform = None 

        self.connect_carla()
        
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz update

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
        # Simple search: find first vehicle. 
        # In a complex sim, you'd filter by role_name attribute if set.
        vehicles = actors.filter('vehicle.*')
        
        if len(vehicles) > 0:
            self.vehicle = vehicles[0]
            self.get_logger().info(f"Attached to vehicle: {self.vehicle.type_id} (ID: {self.vehicle.id})")
        else:
            self.get_logger().warn("No vehicle found yet. Retrying...")
            self.vehicle = None

    def carla_transform_to_ros_pose(self, transform):
        # Carla (Left-Handed: X-fwd, Y-right, Z-up) -> ROS (Right-Handed: X-fwd, Y-left, Z-up)
        # We need to invert Y for location and rotation (pitch and yaw)
        
        x = transform.location.x
        y = -transform.location.y # Invert Y
        z = transform.location.z
        
        # Carla roll/pitch/yaw are in degrees. Convert to radians.
        # Pitch and Yaw are inverted for ROS (right-handed system)
        roll = math.radians(transform.rotation.roll)
        pitch = math.radians(-transform.rotation.pitch) 
        yaw = math.radians(-transform.rotation.yaw)     
        
        # Convert Euler to Quaternion (using ROS conventions)
        # This conversion ensures correct quaternion from ROS Euler
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        
        return (x, y, z), (qx, qy, qz, qw)

    def timer_callback(self):
        if self.vehicle is None or not self.vehicle.is_alive:
            self.find_vehicle()
            return

        try:
            # Gather Data from CARLA
            control = self.vehicle.get_control()
            velocity = self.vehicle.get_velocity() # Global frame velocity
            transform = self.vehicle.get_transform() # Global frame transform
            
            # --- 1. Odometry & TF ---
            current_ros_time = self.get_clock().now().to_msg()
            
            # Convert Position and Orientation from CARLA to ROS format
            (pos_x, pos_y, pos_z), (qx, qy, qz, qw) = self.carla_transform_to_ros_pose(transform)
            
            # Publish Transform (odom -> base_footprint)
            # This makes the base_footprint move in the 'odom' frame
            t = TransformStamped()
            t.header.stamp = current_ros_time
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint' # This is the root of our robot model
            
            t.transform.translation.x = pos_x
            t.transform.translation.y = pos_y
            t.transform.translation.z = pos_z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            
            self.tf_broadcaster.sendTransform(t)
            
            # Publish Odometry Message
            odom = Odometry()
            odom.header.stamp = current_ros_time
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint' # Odometry is usually in child frame's coordinates
            
            odom.pose.pose.position.x = pos_x
            odom.pose.pose.position.y = pos_y
            odom.pose.pose.position.z = pos_z
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            
            # Linear and Angular Velocities for Odom twist (in child_frame_id - base_footprint)
            # Carla's velocity is global, we need to convert it to local frame
            # For simplicity, let's convert global velocity to ROS frame and assume no angular velocity for now
            # Proper conversion involves rotating the global velocity vector by inverse of current orientation
            
            # Convert global linear velocity to ROS convention (invert Y)
            odom.twist.twist.linear.x = velocity.x
            odom.twist.twist.linear.y = -velocity.y
            odom.twist.twist.linear.z = velocity.z
            
            # Angular velocity is harder to get directly from CARLA vehicle object,
            # would require differentiating orientation or using sensor.
            # Leaving as 0 for now.
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0
            
            self.odom_pub.publish(odom)

            # --- 2. Joint States (Wheel Animation) ---
            
            # Speed needs to be relative to the car's forward direction for wheel spin calculation
            # Transform global velocity vector to local vehicle frame
            local_vel_x = (velocity.x * transform.get_forward_vector().x +
                           velocity.y * transform.get_forward_vector().y +
                           velocity.z * transform.get_forward_vector().z)
            
            # Use local_vel_x for wheel spin calculation
            speed_for_wheels = local_vel_x # This is effectively the forward speed

            max_steer_rad = 0.6
            steering_angle = -1.0 * control.steer * max_steer_rad

            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time
            
            wheel_radius = 0.3
            angular_vel = speed_for_wheels / wheel_radius
            self.wheel_rotation_angle += angular_vel * dt
            self.wheel_rotation_angle %= (2 * math.pi)

            msg = JointState()
            msg.header.stamp = current_ros_time
            msg.name = [
                'front_left_steering_joint', 'front_right_steering_joint',
                'front_left_wheel_joint', 'front_right_wheel_joint',
                'rear_left_wheel_joint', 'rear_right_wheel_joint'
            ]
            msg.position = [
                steering_angle, steering_angle,
                self.wheel_rotation_angle, self.wheel_rotation_angle,
                self.wheel_rotation_angle, self.wheel_rotation_angle
            ]
            self.joint_pub.publish(msg)

        except RuntimeError:
            self.get_logger().warn("Vehicle lost, reconnecting...")
            self.vehicle = None
            return

def main(args=None):
    rclpy.init(args=args)
    node = CarlaToRosBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()