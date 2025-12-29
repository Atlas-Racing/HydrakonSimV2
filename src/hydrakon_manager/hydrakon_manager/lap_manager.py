#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
import csv
import math
import time

class LapManager(Node):
    def __init__(self):
        super().__init__('lap_manager')

        self.declare_parameter("path_file", "my_track_path.csv")
        self.declare_parameter("waypoint_spacing", 5.0) # Meters between waypoints
        self.declare_parameter("laps", 3)

        self.path_file = self.get_parameter("path_file").value
        self.spacing = self.get_parameter("waypoint_spacing").value
        self.target_laps = self.get_parameter("laps").value
        self.current_lap = 0
        self.amcl_received = False
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
        # Subscribe to AMCL pose to know when we are localized
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)

        self.waypoints = self.load_and_sample_path(self.path_file, self.spacing)
        
        if not self.waypoints:
            self.get_logger().error("No waypoints loaded! Aborting.")
            return

        self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints. Waiting for AMCL localization...")
        
        # Timer to check for start condition
        self.timer = self.create_timer(1.0, self.check_start_condition)
        self.goal_handle = None

    def amcl_callback(self, msg):
        if not self.amcl_received:
            self.get_logger().info("AMCL Pose received! System is localized.")
            self.amcl_received = True

    def check_start_condition(self):
        if self.amcl_received:
            self.timer.cancel() # Stop checking
            # Give a few more seconds for costmaps to update after localization
            self.get_logger().info("AMCL Ready. Waiting 5s for costmap stabilization...")
            time.sleep(5.0)
            
            self.get_logger().info("Engaging Nav2...")
            self.start_lap()
        else:
            self.get_logger().info("Waiting for /amcl_pose...", throttle_duration_sec=2.0)

    def load_and_sample_path(self, filename, spacing):
        points = []
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                next(reader, None) # Skip header
                for row in reader:
                    if len(row) >= 2:
                        points.append((float(row[0]), float(row[1])))
        except Exception as e:
            self.get_logger().error(f"Failed to read path file: {e}")
            return []

        if not points:
            return []

        # Subsample based on distance
        sampled_poses = []
        last_point = points[0]
        
        # Always add the first point
        sampled_poses.append(self.create_pose(points[0]))

        for p in points[1:]:
            dist = math.hypot(p[0] - last_point[0], p[1] - last_point[1])
            if dist >= spacing:
                sampled_poses.append(self.create_pose(p))
                last_point = p
        
        # Ensure the last point is included to close the loop
        if points[-1] != last_point:
             sampled_poses.append(self.create_pose(points[-1]))

        return sampled_poses

    def create_pose(self, point):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.orientation.w = 1.0 # Default orientation
        return pose

    def start_lap(self):
        if self.current_lap >= self.target_laps:
            self.get_logger().info("All laps completed!")
            return

        self.current_lap += 1
        self.get_logger().info(f"Starting Lap {self.current_lap} / {self.target_laps}")

        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("NavigateThroughPoses action server not available!")
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = self.waypoints
        
        # Re-stamp waypoints
        now = self.get_clock().now().to_msg()
        for p in goal_msg.poses:
            p.header.stamp = now

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted! Racing...')
        self.goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        # Optional: Print progress
        # feedback = feedback_msg.feedback
        # self.get_logger().info(f'Remaining waypoints: {feedback.distance_remaining}')
        pass

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Lap Finished!')
        
        # Start next lap after a brief pause
        time.sleep(1.0)
        self.start_lap()

def main(args=None):
    rclpy.init(args=args)
    node = LapManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
