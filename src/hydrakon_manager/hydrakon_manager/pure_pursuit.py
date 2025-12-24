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

        self.declare_parameter("min_lookahead", 3.0)      # Increased to stabilize steering
        self.declare_parameter("max_lookahead", 6.0)      
        self.declare_parameter("max_steering_angle", 1.0) 
        self.declare_parameter("constant_speed", 0.1)     # Low speed for stability
        self.declare_parameter("min_speed", 0.06)         
        self.declare_parameter("track_width_offset", 2.0) 
        self.declare_parameter("steering_gain", 0.8)      # Reduced to prevent oscillation
        
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
        
        for marker in msg.markers:
            if marker.action == 3: continue
            
            x = marker.pose.position.x
            y = marker.pose.position.y
            
            # Reduce visibility horizon to ignore distant distractions
            # Was 15.0, now 10.0 to focus on immediate track
            if x < 0.0 or x > 10.0: 
                continue
            
            p = np.array([x, y])
            
            r, g, b = marker.color.r, marker.color.g, marker.color.b
            if b > 0.9 and r < 0.1: # Blue (Left)
                blue_cones.append(p)
            elif r > 0.9 and g > 0.9: # Yellow (Right)
                yellow_cones.append(p)
        
        # Sort by x distance
        blue_cones.sort(key=lambda p: p[0])
        yellow_cones.sort(key=lambda p: p[0])
        
        target_point = None
        far_target = None
        
        # --- ROBUST LINE-BASED PLANNING ---
        # Only consider a "Line" if the first two cones are close enough.
        # This prevents connecting a near cone with a far cone (gap in track).
        
        has_blue_line = False
        if len(blue_cones) >= 2:
            dist = np.linalg.norm(blue_cones[1] - blue_cones[0])
            if dist < 5.0: # Only form line if cones are somewhat adjacent
                has_blue_line = True
        
        has_yel_line = False
        if len(yellow_cones) >= 2:
            dist = np.linalg.norm(yellow_cones[1] - yellow_cones[0])
            if dist < 5.0:
                has_yel_line = True
        
        if has_blue_line and has_yel_line:
            # Best Case: Both sides visible. Average the gates.
            # Gate 0
            g0 = (blue_cones[0] + yellow_cones[0]) / 2.0
            # Gate 1
            g1 = (blue_cones[1] + yellow_cones[1]) / 2.0
            
            # Target is blend, biased slightly forward for smoothness
            target_point = 0.4 * g0 + 0.6 * g1
            far_target = g1
            
        elif has_blue_line:
            # Left Turn / Right Side Occluded
            # Use Blue Line shifted to the Right
            b0 = blue_cones[0]
            b1 = blue_cones[1]
            
            # Vector b0 -> b1
            d = b1 - b0
            norm = np.linalg.norm(d)
            if norm < 0.1: # Degenerate
                u = np.array([1.0, 0.0])
            else:
                u = d / norm
            
            # Normal to the Right (Rotate -90: x,y -> y, -x)
            n_right = np.array([u[1], -u[0]])
            
            # Shift by offset to get center path
            m0 = b0 + n_right * self.offset
            m1 = b1 + n_right * self.offset
            
            target_point = 0.4 * m0 + 0.6 * m1
            far_target = m1
            
        elif has_yel_line:
            # Right Turn / Left Side Occluded
            # Use Yellow Line shifted to the Left
            y0 = yellow_cones[0]
            y1 = yellow_cones[1]
            
            d = y1 - y0
            norm = np.linalg.norm(d)
            if norm < 0.1:
                u = np.array([1.0, 0.0])
            else:
                u = d / norm
                
            # Normal to the Left (Rotate +90: x,y -> -y, x)
            n_left = np.array([-u[1], u[0]])
            
            # Shift by offset
            m0 = y0 + n_left * self.offset
            m1 = y1 + n_left * self.offset
            
            target_point = 0.4 * m0 + 0.6 * m1
            far_target = m1
            
        else:
            # FALLBACK: Not enough cones to form a line on either side
            # Try to use closest pair
            if blue_cones and yellow_cones:
                target_point = (blue_cones[0] + yellow_cones[0]) / 2.0
            elif blue_cones:
                # Maintain offset from single blue cone (assume straight)
                target_point = blue_cones[0] + np.array([0.0, -self.offset])
            elif yellow_cones:
                # Maintain offset from single yellow cone
                target_point = yellow_cones[0] + np.array([0.0, self.offset])
                
        if target_point is not None:
            # Low Alpha (0.3) for heavy smoothing/damping
            alpha = 0.3
            smoothed_target = alpha * target_point + (1.0 - alpha) * self.prev_target
            self.prev_target = smoothed_target
            
            self.publish_point(self.target_pub, smoothed_target)
            self.drive_to_target(smoothed_target, far_target)
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
