#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import carla
import math
import numpy as np

class INSNode(Node):
    def __init__(self):
        super().__init__('ins_node')

        # Parameters
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("role_name", "ego_vehicle")
        self.declare_parameter("sensor_tick", 0.05)
        self.declare_parameter("enable_noise", True)
        
        # Noise Parameters (Standard Deviation)
        self.declare_parameter("noise_accel_stddev_x", 0.1)
        self.declare_parameter("noise_accel_stddev_y", 0.1)
        self.declare_parameter("noise_accel_stddev_z", 0.1)
        
        self.declare_parameter("noise_gyro_stddev_x", 0.05)
        self.declare_parameter("noise_gyro_stddev_y", 0.05)
        self.declare_parameter("noise_gyro_stddev_z", 0.05)
        
        # Orientation Estimation Noise
        self.declare_parameter("orientation_stddev", 0.01)

        self.host = self.get_parameter("carla_host").value
        self.port = self.get_parameter("carla_port").value
        self.role_name = self.get_parameter("role_name").value
        self.sensor_tick = self.get_parameter("sensor_tick").value
        self.enable_noise = self.get_parameter("enable_noise").value
        
        self.noise_accel = [
            self.get_parameter("noise_accel_stddev_x").value,
            self.get_parameter("noise_accel_stddev_y").value,
            self.get_parameter("noise_accel_stddev_z").value
        ]
        self.noise_gyro = [
            self.get_parameter("noise_gyro_stddev_x").value,
            self.get_parameter("noise_gyro_stddev_y").value,
            self.get_parameter("noise_gyro_stddev_z").value
        ]
        self.orientation_stddev = self.get_parameter("orientation_stddev").value

        # Publisher
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self.client = None
        self.world = None
        self.vehicle = None
        self.imu_sensor = None

        self.connect_carla()
        
        self.timer = self.create_timer(1.0, self.check_vehicle)

        self.get_logger().info(f"INS Node (IMU Spawner) Started. Noise Enabled: {self.enable_noise}")

    def connect_carla(self):
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(5.0)
            self.world = self.client.get_world()
        except Exception as e:
            self.get_logger().error(f"Carla connection failed: {e}")

    def check_vehicle(self):
        if self.vehicle is not None:
            return

        actors = self.world.get_actors()
        vehicles = actors.filter('vehicle.*')
        
        if len(vehicles) > 0:
            self.vehicle = vehicles[0]
            self.get_logger().info(f"Found vehicle: {self.vehicle.type_id}. Spawning INS/IMU...")
            self.spawn_imu()
        else:
            self.get_logger().warn("Waiting for vehicle to spawn INS...")

    def spawn_imu(self):
        try:
            bp = self.world.get_blueprint_library().find('sensor.other.imu')
            bp.set_attribute('sensor_tick', str(self.sensor_tick))
            
            if self.enable_noise:
                bp.set_attribute('noise_accel_stddev_x', str(self.noise_accel[0]))
                bp.set_attribute('noise_accel_stddev_y', str(self.noise_accel[1]))
                bp.set_attribute('noise_accel_stddev_z', str(self.noise_accel[2]))
                
                bp.set_attribute('noise_gyro_stddev_x', str(self.noise_gyro[0]))
                bp.set_attribute('noise_gyro_stddev_y', str(self.noise_gyro[1]))
                bp.set_attribute('noise_gyro_stddev_z', str(self.noise_gyro[2]))
            else:
                bp.set_attribute('noise_accel_stddev_x', '0.0')
                bp.set_attribute('noise_accel_stddev_y', '0.0')
                bp.set_attribute('noise_accel_stddev_z', '0.0')
                bp.set_attribute('noise_gyro_stddev_x', '0.0')
                bp.set_attribute('noise_gyro_stddev_y', '0.0')
                bp.set_attribute('noise_gyro_stddev_z', '0.0')
            
            transform = carla.Transform(carla.Location(x=0, z=0))
            self.imu_sensor = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
            
            self.imu_sensor.listen(self.imu_callback)
            self.get_logger().info("INS/IMU Sensor Spawned and Listening.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to spawn IMU: {e}")

    def imu_callback(self, data):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" 
        
        transform = self.imu_sensor.get_transform()
        
        roll = math.radians(transform.rotation.roll)
        pitch = math.radians(-transform.rotation.pitch)
        yaw = math.radians(-transform.rotation.yaw)
        
        if self.enable_noise:
            roll += np.random.normal(0, self.orientation_stddev)
            pitch += np.random.normal(0, self.orientation_stddev)
            yaw += np.random.normal(0, self.orientation_stddev)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        
        msg.angular_velocity.x = data.gyroscope.x
        msg.angular_velocity.y = -data.gyroscope.y
        msg.angular_velocity.z = -data.gyroscope.z 
        
        msg.linear_acceleration.x = data.accelerometer.x
        msg.linear_acceleration.y = -data.accelerometer.y
        msg.linear_acceleration.z = data.accelerometer.z
        
        if self.enable_noise:
            orient_var = self.orientation_stddev ** 2
            msg.orientation_covariance = [
                orient_var, 0.0, 0.0,
                0.0, orient_var, 0.0,
                0.0, 0.0, orient_var
            ]
            msg.angular_velocity_covariance = [
                self.noise_gyro[0]**2, 0.0, 0.0,
                0.0, self.noise_gyro[1]**2, 0.0,
                0.0, 0.0, self.noise_gyro[2]**2
            ]
            msg.linear_acceleration_covariance = [
                self.noise_accel[0]**2, 0.0, 0.0,
                0.0, self.noise_accel[1]**2, 0.0,
                0.0, 0.0, self.noise_accel[2]**2
            ]
        else:
            msg.orientation_covariance = [0.0] * 9
            msg.angular_velocity_covariance = [0.0] * 9
            msg.linear_acceleration_covariance = [0.0] * 9
        
        self.imu_pub.publish(msg)

    def destroy(self):
        if self.imu_sensor:
            self.imu_sensor.destroy()

def main(args=None):
    rclpy.init(args=args)
    node = INSNode()
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