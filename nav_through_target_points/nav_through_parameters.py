#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Aug  5 02:02:57 2026

@author: Maliha
"""

import rclpy
from rclpy.node import Node

import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class NavigationParameters(Node):
    def __init__(self):
        super().__init__("multi_point_nav")

        self.get_logger().info("Multipoint node has started! :)")
        self.navigator = BasicNavigator()
        self.get_logger().info("Mwaiting for Nav2 to become active... :)")
        self.navigator.waitUntilNav2Active()
        
        self.waypoints = self.get_user_points()
        self.get_logger().info(f"Collected points: {self.waypoints}")
        self.navigate_to_all_points()
        
    def get_user_points(self):
        points = []
        print("\n Enter three way points: ")
        while len(points) < 3:
            point_num = len(points) + 1
            
            try: 
                x = float(input(f"Enter the x coordinate for {point_num}"))
                y = float(input(f"Enter the y coordinate for {point_num}"))
                points.append((x,y))
                print(f"Point {point_num} saved: ({x}, {y})\n")
                
            except ValueError:
                print("Invalid input! Please enter numbers only.\n")
        return points
    
    def navigate_to_all_points(self):
        for idx, (x,y) in enumerate(self.waypoints):
            self.get_logger().info(f"Moving to Waypoint {idx + 1}: X = {x}, Y = {y}")
        
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0


            self.navigator.goToPose(goal_pose)
            success = self.monitor_progress(idx + 1)   
            
        
            if success:
                self.get_logger().info("Waiting here for 3 seconds...")
                time.sleep(3.0) # <--- This line freezes the script for exactly 3 seconds
        
            
            
                
    def monitor_progress(self, point_number):
                while not self.navigator.isTaskComplete():
                    feedback = self.navigator.getFeedback()
                    if feedback:
                        self.get_logger().info(f"Distance remaining to point {point_number}: {feedback.distance_remaining:.2f}m",
                            throttle_duration_sec=2.0 # Stops the terminal from being flooded
                        )
                        
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"Successfully reached Point {point_number}!")
                    return True
                elif result == TaskResult.CANCELED:
                    self.get_logger().warn(f"Navigation to Point {point_number} was canceled.")
                    return False
                elif result == TaskResult.FAILED:
                    self.get_logger().error(f"Failed to reach Point {point_number}. Skipping to next.")
                    return False
        
def main(args=None):
    rclpy.init(args=args)
    node = NavigationParameters()
    
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
