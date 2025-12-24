#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import carla
import pygame
import numpy as np
import threading
import time

class ManualControlNode(Node):
    def __init__(self):
        super().__init__('manual_control_node')
        
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("role_name", "ego_vehicle")
        
        self.host = self.get_parameter("carla_host").value
        self.port = self.get_parameter("carla_port").value
        self.role_name = self.get_parameter("role_name").value
        
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        self.client = None
        self.world = None
        self.vehicle = None
        self.camera = None
        
        self.image_width = 800
        self.image_height = 600
        self.latest_image = None
        self.lock = threading.Lock()
        
        self.get_logger().info("Manual Control Node Started. Waiting for Carla...")

    def cmd_vel_callback(self, msg):
        if not self.vehicle:
            return

        # If user is pressing keys, ignore remote command to avoid fighting
        # We can't easily check pygame state here, so we rely on main loop overriding us if needed.
        # But since main loop runs at 60Hz and this callback runs async, 
        # main loop will likely win if it applies control every frame.
        # So we trust main loop to ONLY apply control if keys are pressed.
        
        control = carla.VehicleControl()
        control.steer = -msg.angular.z
        control.steer = max(-1.0, min(1.0, control.steer))

        vx = msg.linear.x
        if vx >= 0.0:
            control.throttle = min(1.0, vx)
            control.brake = 0.0
            control.reverse = False
        else:
            control.throttle = min(1.0, abs(vx))
            control.brake = 0.0
            control.reverse = True
        
        control.hand_brake = False
        control.manual_gear_shift = False
        
        self.vehicle.apply_control(control)

    def connect_carla(self):
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(5.0)
            self.world = self.client.get_world()
            self.vehicle = self.find_vehicle()
            
            if self.vehicle:
                self.setup_camera()
                self.get_logger().info(f"Connected to vehicle: {self.vehicle.type_id}")
                return True
            else:
                self.get_logger().warn("Vehicle not found. Retrying...")
                return False
        except Exception as e:
            self.get_logger().error(f"Carla connection failed: {e}")
            return False

    def find_vehicle(self):
        actors = self.world.get_actors()
        vehicles = actors.filter('vehicle.*')
        if len(vehicles) > 0:
            return vehicles[0]
        return None

    def setup_camera(self):
        bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(self.image_width))
        bp.set_attribute('image_size_y', str(self.image_height))
        bp.set_attribute('fov', '90')
        
        transform = carla.Transform(carla.Location(x=-6.5, z=2.5), carla.Rotation(pitch=-15))
        
        self.camera = self.world.spawn_actor(bp, transform, attach_to=self.vehicle)
        self.camera.listen(self.process_image)

    def process_image(self, image):
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        
        with self.lock:
            self.latest_image = array

    def destroy(self):
        if self.camera:
            self.camera.destroy()
            self.camera = None

def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    
    pygame.init()
    pygame.font.init()
    display = pygame.display.set_mode((node.image_width, node.image_height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("Hydrakon Monitor (Manual + Subscriber)")
    font = pygame.font.SysFont('monospace', 18)
    clock = pygame.time.Clock()
    
    reverse = False
    
    try:
        running = True
        while running:
            clock.tick(60)
            
            if node.vehicle is None:
                if not node.connect_carla():
                    time.sleep(1.0)
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            running = False
                    continue

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_q:
                        reverse = not reverse
            
            if node.vehicle:
                keys = pygame.key.get_pressed()
                control = carla.VehicleControl()
                
                # WASD Logic
                if keys[pygame.K_w]:
                    control.throttle = 1.0
                else:
                    control.throttle = 0.0
                    
                if keys[pygame.K_s]:
                    control.brake = 1.0
                else:
                    control.brake = 0.0
                    
                if keys[pygame.K_a]:
                    control.steer = -0.6
                elif keys[pygame.K_d]:
                    control.steer = 0.6
                else:
                    control.steer = 0.0
                    
                if keys[pygame.K_SPACE]:
                    control.hand_brake = True
                else:
                    control.hand_brake = False
                    
                control.reverse = reverse
                control.manual_gear_shift = False
                
                # Check if any driving key is pressed
                any_key_pressed = any(keys[k] for k in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d, pygame.K_SPACE])
                
                if any_key_pressed:
                    node.vehicle.apply_control(control)

            # Retrieve current control state for HUD (reflects either manual or ROS input)
            current_control = carla.VehicleControl()
            if node.vehicle:
                current_control = node.vehicle.get_control()

            with node.lock:
                if node.latest_image is not None:
                    surface = pygame.surfarray.make_surface(node.latest_image.swapaxes(0, 1))
                    display.blit(surface, (0, 0))
            
            hud_text = [
                f"Throttle: {current_control.throttle:.1f}",
                f"Steer: {current_control.steer:.1f}",
                f"Brake: {current_control.brake:.1f}",
                f"Reverse: {current_control.reverse}",
                f"FPS: {clock.get_fps():.0f}",
                "Mode: Manual (WASD) & Subscriber (/cmd_vel)",
                "Controls: W=Gas, S=Brake, A/D=Steer, Space=Handbrake, Q=Toggle Reverse"
            ]
            
            for i, line in enumerate(hud_text):
                text_surf = font.render(line, True, (255, 255, 255))
                display.blit(text_surf, (10, 10 + i * 20))
                
            pygame.display.flip()
            
            rclpy.spin_once(node, timeout_sec=0.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        pygame.quit()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()