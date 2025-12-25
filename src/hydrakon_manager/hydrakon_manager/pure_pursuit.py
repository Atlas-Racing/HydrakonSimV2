#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PointStamped
import math
import numpy as np
import subprocess
import os
import time

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        # Tuned parameters for stability and completion
        self.declare_parameter("min_lookahead", 2.0)
        self.declare_parameter("max_lookahead", 5.0)
        self.declare_parameter("max_steering_angle", 0.7)  # ~40 degrees
        self.declare_parameter("constant_speed", 0.1)      # Slow but moving
        self.declare_parameter("min_speed", 0.05)
        self.declare_parameter("track_width_offset", 1.8)  # Slightly narrower offset for tighter control
        self.declare_parameter("steering_gain", 2.0)       # Added missing declaration
        self.declare_parameter("vision_horizon", 15.0)
        self.declare_parameter("single_side_offset_multiplier", 7.0)

        self.min_lookahead = self.get_parameter("min_lookahead").value
        self.max_lookahead = self.get_parameter("max_lookahead").value
        self.max_steer = self.get_parameter("max_steering_angle").value
        self.target_speed = self.get_parameter("constant_speed").value
        self.min_speed = self.get_parameter("min_speed").value
        self.offset = self.get_parameter("track_width_offset").value
        self.steering_gain = self.get_parameter("steering_gain").value
        self.vision_horizon = self.get_parameter("vision_horizon").value
        self.offset_multiplier = self.get_parameter("single_side_offset_multiplier").value

        # Subscribers
        # Use Camera Cones directly (from cone_locator)
        self.marker_sub = self.create_subscription(MarkerArray, '/camera/cone_markers', self.marker_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.target_pub = self.create_publisher(PointStamped, '/pure_pursuit/target', 10)

        self.current_speed = 0.0
        
        # Lap Completion State Machine
        # 0: START_ZONE (Waiting to leave start area)
        # 1: RACING     (Driving, looking for finish)
        # 2: FINISHED   (Stop and Save)
        # 3: LAP_COOLDOWN (Just crossed finish line, ignore cones briefly)
        self.state = 0 
        self.start_time = self.get_clock().now()
        self.cooldown_start_time = None
        self.lap_count = 0
        self.target_laps = 2
        
        self.get_logger().info("Robust Pure Pursuit Node Started")

    def odom_callback(self, msg):
        self.current_speed = msg.twist.twist.linear.x

    def marker_callback(self, msg):
        if self.state == 2: # FINISHED
            self.stop()
            return

        if not msg.markers:
            self.stop()
            return

        # Update dynamic parameters if changed
        if self.has_parameter("single_side_offset_multiplier"):
             self.offset_multiplier = self.get_parameter("single_side_offset_multiplier").value
        
        if self.has_parameter("vision_horizon"):
             self.vision_horizon = self.get_parameter("vision_horizon").value

        blue_cones = []
        yellow_cones = []
        orange_cones = []
        
        rejected_range = 0
        rejected_color = 0
        total_cones = 0
        
        for marker in msg.markers:
            if marker.action == 3: continue 
            total_cones += 1
            
            x = marker.pose.position.x
            y = marker.pose.position.y
            
            # Relaxed filtering: 
            # Accept cones starting from -1.0 (slightly behind camera plane to catch noise)
            if x < -1.0 or x > self.vision_horizon: 
                rejected_range += 1
                continue
            
            p = np.array([x, y])
            
            # Color classification
            r, g, b = marker.color.r, marker.color.g, marker.color.b
            if b > 0.9 and r < 0.1: # Blue (Left)
                blue_cones.append(p)
            elif r > 0.9 and g > 0.9: # Yellow (Right)
                yellow_cones.append(p)
            elif r > 0.9 and 0.4 < g < 0.6: # Orange
                orange_cones.append(p)
            else:
                rejected_color += 1
        
        # State Machine Logic
        current_time = self.get_clock().now()
        time_elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        if self.state == 0: # START_ZONE
            # We assume we start seeing orange cones. 
            # We switch to RACING only when we DON'T see them anymore and have driven a bit.
            if len(orange_cones) == 0 and time_elapsed > 5.0:
                self.get_logger().info("Left Start Zone. RACING MODE ACTIVE.")
                self.state = 1
                
        elif self.state == 1: # RACING
            # Now if we see orange cones, it's the finish line.
            # Only consider cones that are "really near" (< 5.0m) to avoid premature stopping
            close_orange_cones = [p for p in orange_cones if p[0] < 5.0]
            
            if len(close_orange_cones) >= 2:
                self.lap_count += 1
                self.get_logger().info(f"LAP {self.lap_count} COMPLETED!")
                
                if self.lap_count >= self.target_laps:
                    self.get_logger().info("Target laps reached. Stopping and Saving Map...")
                    self.stop()
                    self.state = 2
                    
                    # Wait for map update
                    time.sleep(2.0)

                    # Auto-save Map
                    map_path = os.path.join(os.path.expanduser("~"), "HydrakonSimV2", "my_track_map")
                    try:
                        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path]
                        self.get_logger().info(f"Executing: {' '.join(cmd)}")
                        subprocess.run(cmd, check=True)
                        self.get_logger().info(f"Map successfully saved to {map_path}")
                    except subprocess.CalledProcessError as e:
                        self.get_logger().error(f"Failed to save map: {e}")
                    return
                else:
                    self.get_logger().info("Starting next lap. Entering Cooldown...")
                    self.state = 3
                    self.cooldown_start_time = current_time

        elif self.state == 3: # LAP_COOLDOWN
             cooldown_elapsed = (current_time - self.cooldown_start_time).nanoseconds / 1e9
             if cooldown_elapsed > 5.0:
                 self.get_logger().info("Cooldown finished. Resuming Race Mode.")
                 self.state = 1

        target_point = None
        
        # Strategy: "Anchor to the furthest visible gate"
        # This provides the most stable path by looking as far ahead as possible
        # within the reliable detection range.
        
        desired_x = self.max_lookahead
        
        # Helper: Find cone closest to the desired lookahead X distance
        def get_best_cone(cones, target_x):
            if not cones: return None
            # Minimize absolute difference in X
            return min(cones, key=lambda p: abs(p[0] - target_x))

        b_best = get_best_cone(blue_cones, desired_x)
        y_best = get_best_cone(yellow_cones, desired_x)
        
        if b_best is not None and y_best is not None:
            # Best case: We see both sides. Aim for the middle.
            target_point = (b_best + y_best) / 2.0
            
        elif b_best is not None:
            # Only Left side visible. Keep offset to the Right.
            # Blue cones are at +Y. We want to be at Y - offset.
            # Increased offset to force sharper turn into the track
            target_point = b_best + np.array([0.0, -self.offset * self.offset_multiplier])
            
        elif y_best is not None:
            # Only Right side visible. Keep offset to the Left.
            # Yellow cones are at -Y. We want to be at Y + offset.
            # Increased offset to force sharper turn into the track
            target_point = y_best + np.array([0.0, self.offset * self.offset_multiplier])
            
        else:
            # No valid cones found. Safety stop.
            self.get_logger().warn(f"No valid cones! Total: {total_cones}, Range Reject: {rejected_range}, Color Reject: {rejected_color}")
            self.stop()
            return

        # Visualize target
        self.publish_point(self.target_pub, target_point)
        
        # Execute Control
        self.drive_to_target(target_point)

    def publish_point(self, publisher, point):
        t_msg = PointStamped()
        t_msg.header.stamp = self.get_clock().now().to_msg()
        t_msg.header.frame_id = "base_link"
        t_msg.point.x = float(point[0])
        t_msg.point.y = float(point[1])
        publisher.publish(t_msg)

    def drive_to_target(self, target):
        x = target[0]
        y = target[1]
        dist = math.hypot(x, y)
        
        # Adaptive Lookahead for steering calculation
        # If target is far, use its distance. 
        # If target is too close, clamp L_d to min_lookahead to avoid unstable high-gain steering.
        L_d = max(self.min_lookahead, dist)
        
        wheelbase = 2.8 
        
        # Pure Pursuit Arc: curvature = 2*y / L_d^2
        # Steering angle = atan(curvature * wheelbase)
        steering_angle = math.atan((2.0 * wheelbase * y) / (L_d * L_d))
        steering_angle *= self.steering_gain
        steering_angle = max(-self.max_steer, min(self.max_steer, steering_angle))
        
        # Speed Control
        # Slow down based on steering sharpness.
        # ratio: 0.0 (straight) -> 1.0 (max turn)
        ratio = abs(steering_angle) / self.max_steer
        
        # Linearly interpolate between max and min speed
        throttle = self.target_speed * (1.0 - ratio) + self.min_speed * ratio
        
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
