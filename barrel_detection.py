#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug  4 17:47:50 2026
@author: maliha
"""
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math


class BarrelNavigator(Node):
    def __init__(self):
        super().__init__("barrel_navigator")
        self.map_pgm_path = "arena_map.pgm"
        self.map_yaml_path = "arena_map.yaml"
        self.get_logger().info("Reading map for barrel identification!")
        
        
        self.resolution, self.origin_x, self.origin_y = self.parse_map_yaml(self.map_yaml_path)
        self.get_logger().info("Scanning map for barrel")
        target_pixel = self.detect_barrel(self.map_pgm_path)
        
        self.stop_distance = 0.4
        
        if target_pixel is None:
            self.get_logger().error("Could not find the barrel!")
            return
        barrel_x, barrel_y = self.pixel_to_world(target_pixel[0], target_pixel[1])
        self.get_logger().info(f"Barrel is located at, x = {world_x: .2f}m, y = {world_y:.2f}m")
        
        self.navigator = BasicNavigator()
        self.get_logger().info("Connecting to Nav2")
        self.navigator.waitUntilNav2Active()
        
        
        initial_pose = self.navigator.getRobotPose()
        robot_x = initial_pose.pose.position.x
        robot_y = initial_pose.pose.position.y
        
        dx = barrel_x - robot_x
        dy = barrel_y - robot_y
        
        distance_to_barrel = math.sqrt(dx**2 + dy**2)
        angle_to_barrel = math.atan2(dy, dx)
        
        if distance_to_barrel <= self.stop_distance:
            self.get_logger().warn('Robot is already closer than the requested stopping distance!')
            return

        goal_x = barrel_x - (self.stop_distance * math.cos(angle_to_barrel))
        goal_y = barrel_y - (self.stop_distance * math.sin(angle_to_barrel))
        q_z = math.sin(angle_to_barrel / 2.0)
        q_w = math.cos(angle_to_barrel / 2.0)
        
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = goal_x
        goal_pose.pose.position.y = goal_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        
        self.get_logger().info('Driving to the barrel...')
        self.navigator.goToPose(goal_pose)   
        
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f"Distance to barrel: {feedback.distance_remaining:.2f} meters")
        
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal is reached')
            
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Navigation cancelled manually :(')
            
        elif result == TaskResult.FAILED:
            self.get_logger().error('Navigation failed :/')
            
            
    def parse_map_yaml(self, yaml_path):
            with open(yaml_path, "r") as file:
                data = yaml.safe_load(file)
                
            resolution = data["resolution"]
            origin = data["origin"]
            return resolution, origin[0], origin[1]
        
    def detect_barrel(self, pgm_path):
            
        img = cv2.imread(pgm_path, 0)
        if img is None:
            return None
            
        _, binary = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
       
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
        clean_mask = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        barrel_radius_m = 0.3/2.0
        expected_area_m2 = np.pi * (barrel_radius_m **2)
        expected_pixel_area = expected_area_m2 / (self.resolution ** 2)
        
        
        min_area = expected_pixel_area * 0.6
        max_area = expected_pixel_area * 1.4
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if min_area <= area <= max_area:
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cX = int(M["m10"]/ M["m00"])
                    cY = int(M["m01"]/ M["m00"])
                    
                    height, _ = img.shape
                    ros_pixel_y = height - cY
                    
                    return (cX, ros_pixel_y)
        return None
    
    def pixel_to_world(self, pixel_x, pixel_y):
        world_x = self.origin_x + (pixel_x * self.resolution)
        world_y = self.origin_y + (pixel_y * self.resolution)
        
        return world_x, world_y
    
    
def main(args=None):
    rclpy.init(args=args)
    node = BarrelNavigator()
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()