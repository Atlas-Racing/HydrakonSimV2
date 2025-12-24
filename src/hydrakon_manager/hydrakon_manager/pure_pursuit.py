#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
import math
import numpy as np

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.declare_parameter("min_lookahead", 1.5)      # Reduced for sharper turns
        self.declare_parameter("max_lookahead", 6.0)      
        self.declare_parameter("max_steering_angle", 1.0) 
        self.declare_parameter("constant_speed", 0.15)    # Slower speed
        self.declare_parameter("min_speed", 0.08)         # Slower cornering
        self.declare_parameter("track_width_offset", 2.0) 
        self.declare_parameter("steering_gain", 1.4)      # Increased for tighter radius
        
        self.min_lookahead = self.get_parameter("min_lookahead").value
        self.max_lookahead = self.get_parameter("max_lookahead").value
        self.max_steer = self.get_parameter("max_steering_angle").value
        self.target_speed = self.get_parameter("constant_speed").value
        self.min_speed = self.get_parameter("min_speed").value
        self.offset = self.get_parameter("track_width_offset").value
        self.steering_gain = self.get_parameter("steering_gain").value

        # Subscribers
        # Use Fused Cones (Already in base_link)
        self.marker_sub = self.create_subscription(MarkerArray, '/fusion/cone_markers', self.marker_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(PointStamped, '/pure_pursuit/target', 10)

        self.current_speed = 0.0
        self.prev_target = np.array([3.0, 0.0]) 
        
        self.get_logger().info("Adaptive Pure Pursuit Node Started (Using Sensor Fusion)")

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def marker_callback(self, msg):
        if not msg.markers:
            return

        blue_cones = []
        yellow_cones = []
        orange_cones = []
        
        # Cones are already in base_link frame from fusion node
        for marker in msg.markers:
            if marker.action == 3: # DELETEALL
                continue
                
            x_final = marker.pose.position.x
            y_final = marker.pose.position.y
            
            pos = np.array([x_final, y_final])
            
            # Visibility window
            if x_final < 0.5 or x_final > 20.0: 
                continue
            
            r, g, b = marker.color.r, marker.color.g, marker.color.b
            if r > 0.9 and g > 0.9 and b < 0.1: # Yellow
                yellow_cones.append(pos)
            elif b > 0.9 and r < 0.1 and g < 0.1: # Blue
                blue_cones.append(pos)
            elif r > 0.9 and g > 0.4 and b < 0.1: # Orange
                orange_cones.append(pos)
        
        # Sort by distance
        blue_cones.sort(key=lambda p: p[0])
        yellow_cones.sort(key=lambda p: p[0])
        orange_cones.sort(key=lambda p: p[0])
        
        raw_target = None
        far_target = None
        
        # --- TARGET SELECTION ---
        if orange_cones:
            raw_target = orange_cones[0]
            
        elif blue_cones and yellow_cones:
            # DUAL MODE (Straight/Curve)
            # 1. Near Target (Position) - Closest 2 cones
            b_near = np.mean(blue_cones[:2], axis=0)
            y_near = np.mean(yellow_cones[:2], axis=0)
            near_midpoint = (b_near + y_near) / 2.0
            
            # 2. Far Target (Heading) - Cones further down (index 2-4 if exist)
            # Use up to 5th cone for far lookahead
            b_far_list = blue_cones[2:5] if len(blue_cones) > 2 else blue_cones[-1:]
            y_far_list = yellow_cones[2:5] if len(yellow_cones) > 2 else yellow_cones[-1:]
            
            if b_far_list and y_far_list:
                b_far = np.mean(b_far_list, axis=0)
                y_far = np.mean(y_far_list, axis=0)
                far_midpoint = (b_far + y_far) / 2.0
                
                # Weighted Blend: 40% Position (Near), 60% Heading (Far)
                # This aligns the car with the track direction
                raw_target = 0.4 * near_midpoint + 0.6 * far_midpoint
                far_target = far_midpoint # For curvature check
            else:
                raw_target = near_midpoint
            
        elif blue_cones:
            # SINGLE MODE (Left)
            # Aim parallel to the cone line
            raw_target = blue_cones[0] + np.array([0, -self.offset])
            if len(blue_cones) > 2:
                far_target = blue_cones[-1] + np.array([0, -self.offset])
            
        elif yellow_cones:
            # SINGLE MODE (Right)
            raw_target = yellow_cones[0] + np.array([0, self.offset])
            if len(yellow_cones) > 2:
                far_target = yellow_cones[-1] + np.array([0, self.offset])
        
        if raw_target is not None:
            # Alpha filter
            alpha = 0.5
            target_point = alpha * raw_target + (1.0 - alpha) * self.prev_target
            self.prev_target = target_point
            
            self.publish_point(self.target_pub, target_point)
            self.drive_to_target(target_point, far_target)
        else:
            self.stop()

    def publish_point(self, publisher, point):
        t_msg = PointStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = "base_link"
        t_msg.point.x = float(point[0])
        t_msg.point.y = float(point[1])
        publisher.publish(t_msg)

    def drive_to_target(self, target, far_target):
        x = target[0]
        y = target[1]
        dist = math.hypot(x, y)
        
        # --- ADAPTIVE LOOKAHEAD ---
        # Scale lookahead with current speed
        # Low speed (0.1) -> Min Lookahead (2.0m)
        # High speed (0.22) -> Max Lookahead (6.0m)
        speed_ratio = (self.current_speed - self.min_speed) / (self.target_speed - self.min_speed + 1e-5)
        speed_ratio = np.clip(speed_ratio, 0.0, 1.0)
        
        current_lookahead = self.min_lookahead + speed_ratio * (self.max_lookahead - self.min_lookahead)
        
        # Effective lookahead is max of calculated and actual target dist
        L_d = max(current_lookahead, dist)
        
        wheelbase = 2.8 
        
        # Steering Calculation
        steering_angle = math.atan((2.0 * wheelbase * y) / (L_d * L_d))
        steering_angle *= self.steering_gain
        steering_angle = max(-self.max_steer, min(self.max_steer, steering_angle))
        
        # --- PREDICTIVE SPEED CONTROL ---
        future_curvature = 0.0
        if far_target is not None:
            fx, fy = far_target[0], far_target[1]
            f_dist = math.hypot(fx, fy)
            if f_dist > 1.0:
                future_curvature = abs(math.atan((2.0 * wheelbase * fy) / (f_dist * f_dist)))
        
        current_curvature = abs(steering_angle)
        max_curve_detected = max(current_curvature, future_curvature)
        
        # Reduce speed on curves
        speed_factor = 1.0 - (max_curve_detected / self.max_steer) * 0.7
        speed_factor = max(0.0, speed_factor)
        
        throttle = self.target_speed * speed_factor
        throttle = max(self.min_speed, throttle)
        
        twist = Twist()
        twist.linear.x = throttle
        twist.angular.z = steering_angle 
        self.cmd_vel_pub.publish(twist)

    def stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
