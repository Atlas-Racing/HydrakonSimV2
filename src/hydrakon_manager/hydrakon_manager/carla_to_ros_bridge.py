#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
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

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.latest_angular_vel = None
        self.latest_orientation = None
        self.latest_orientation_covariance = [0.0] * 9
        self.latest_angular_vel_covariance = [0.0] * 9

        self.client = None
        self.world = None
        self.vehicle = None
        
        self.wheel_rotation_angle = 0.0
        self.last_time = time.time()
        
        self.connect_carla()
        
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("Carla-to-ROS Bridge Node Started.")

    def imu_callback(self, msg):
        self.latest_angular_vel = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        )
        self.latest_orientation = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.latest_orientation_covariance = list(msg.orientation_covariance)
        self.latest_angular_vel_covariance = list(msg.angular_velocity_covariance)

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

    def carla_transform_to_ros_pose(self, transform):

        x = transform.location.x
        y = -transform.location.y
        z = transform.location.z
        
        roll = math.radians(transform.rotation.roll)
        pitch = math.radians(-transform.rotation.pitch) 
        yaw = math.radians(-transform.rotation.yaw)     
        
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
            control = self.vehicle.get_control()
            velocity = self.vehicle.get_velocity()
            transform = self.vehicle.get_transform()
            
            current_ros_time = self.get_clock().now().to_msg()
            
            (pos_x, pos_y, pos_z), (qx, qy, qz, qw) = self.carla_transform_to_ros_pose(transform)
            
            
            if not self.latest_orientation:
                return

            qx, qy, qz, qw = self.latest_orientation
            
            t = TransformStamped()
            t.header.stamp = current_ros_time
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            
            t.transform.translation.x = pos_x
            t.transform.translation.y = pos_y
            t.transform.translation.z = pos_z
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            
            self.tf_broadcaster.sendTransform(t)
            
            odom = Odometry()
            odom.header.stamp = current_ros_time
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'
            
            odom.pose.pose.position.x = pos_x
            odom.pose.pose.position.y = pos_y
            odom.pose.pose.position.z = pos_z
            odom.pose.pose.orientation.x = qx
            odom.pose.pose.orientation.y = qy
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            
            for i in range(3):
                for j in range(3):
                    odom.pose.covariance[(i + 3) * 6 + (j + 3)] = self.latest_orientation_covariance[i * 3 + j]
                    odom.twist.covariance[(i + 3) * 6 + (j + 3)] = self.latest_angular_vel_covariance[i * 3 + j]
            
            odom.pose.covariance[0] = 0.01  # x variance
            odom.pose.covariance[7] = 0.01  # y variance
            odom.pose.covariance[14] = 0.01 # z variance
            
            # For linear velocity X, Y, Z
            odom.twist.covariance[0] = 0.01  # x variance
            odom.twist.covariance[7] = 0.01  # y variance
            odom.twist.covariance[14] = 0.01 # z variance
            
            odom.twist.twist.linear.x = velocity.x
            odom.twist.twist.linear.y = -velocity.y
            odom.twist.twist.linear.z = velocity.z
            
            if self.latest_angular_vel:
                odom.twist.twist.angular.x = self.latest_angular_vel[0]
                odom.twist.twist.angular.y = self.latest_angular_vel[1]
                odom.twist.twist.angular.z = self.latest_angular_vel[2]
            else:
                odom.twist.twist.angular.x = 0.0
                odom.twist.twist.angular.y = 0.0
                odom.twist.twist.angular.z = 0.0
            
            
            self.odom_pub.publish(odom)

            local_vel_x = (velocity.x * transform.get_forward_vector().x +
                           velocity.y * transform.get_forward_vector().y +
                           velocity.z * transform.get_forward_vector().z)
            
            speed_for_wheels = local_vel_x

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