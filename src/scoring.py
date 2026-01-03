#!/usr/bin/env python3
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseArray, Point
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Int64MultiArray
from pushback_sim.msg import Ball, BallArray
from typing import Tuple

'''
pushback_sim/Ball
---
string object_name
int64 id
geometry_msgs/Point location
'''

class Scoring(Node):
    def __init__(self):
        super().__init__('scoring')
        self.create_subscription(Joy, '/joy', self.controller_callback, 10)
        self.create_subscription(BallArray, '/object_locations', self.object_location_callback, 10)

        self.collisions = self.create_publisher(Ball, '/robot_collisions', 10)

        # robot location / pose subscriber
        self.create_subscription(PoseArray, '/otto_pose', self.robot_pose_callback, 10)

        self.robot_hopper = []

        self.tol = (0.2, 0.2, 0.1)
        self.z_tol = 0.01

    def controller_callback(self, msg:Joy):
        # self.get_logger().info('getting controller messages')
        # check controller input for the intake / scoring buttons being pressed
        if msg.buttons[0] == 1: # ball out
            location = self.check_location()
            self.get_logger().info(f'Otto is at goal ID: {location}')
        elif msg.buttons[1] == 1: # activate intake
            # intake funtion
            self.check_collision()

    def robot_pose_callback(self, msg:PoseArray):
        self.robot_x = msg.poses[-1].position.x
        self.robot_y = msg.poses[-1].position.y

        quat = msg.poses[-1].orientation
        rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
        euler = rotation.as_euler('xyz') # THIS IS IN RADIANS!
        self.robot_r = euler[2] # get the yaw (z rotation) value from the returned array

        # self.get_logger().info(f" ROBOT POSE: x:{self.robot_x} y:{self.robot_y} r:{self.robot_r}")
    
    def object_location_callback(self, msg:BallArray):
        self.objects=msg
        # TODO: add in filtering for z height (goals vs non goals)
            
    def check_collision(self):
        h, k = self.robot_x, self.robot_y
        th = self.robot_r
        r = 0.025

        # parametric equations to find a reference point based on the robot's pose
        x = h + r * np.cos(th)
        y = k + r * np.sin(th)
        self.get_logger().info(f'[CHECK_COLLISION] reference point: x: {x}, y: {y}')

        # validate ball location helper method
        def intake_zone(ref_x, ref_y, ball_pos:Point):
            z_t = 0.05
            x_error = abs(ball_pos.x - ref_x)
            y_error = abs(ball_pos.y - ref_y)

            if ball_pos.z < z_t and x_error < 0.10 and y_error < 0.10:
                return True
            else:
                return False
        
        # check all objects for a collision
        for ball in self.objects.object_array:
            valid = intake_zone(x, y, ball.location)
            if valid:
                self.get_logger().info(f"[CHECK_COLLISION] collected ball! {ball.id}")
                self.collisions.publish(ball)
        
    def in_bounds(self, pose:Tuple, goal:Tuple):
        error_x = abs(pose[0] - goal[0])
        error_y = abs(pose[1] - goal[1])
       # error_r = abs(pose[2] - goal[2])

        if error_x < self.tol[0] and error_y < self.tol[1]: # add rotational error back soon ...
            return True
        else:
            return False
    
    def get_quadrant(self):
        if self.robot_x >= 0 and self.robot_y >= 0: # quadrant 1
            return 1
        elif self.robot_x < 0 and self.robot_y >= 0: # quadrant 2
            return 2
        elif self.robot_x < 0 and self.robot_y < 0: # quadrant 3
            return 3
        elif self.robot_x >= 0 and self.robot_y < 0: # quadrant 4
            return 4
        
    def check_location(self):
        robo_pose = (self.robot_x, self.robot_y)
        q = self.get_quadrant()

        long_goal_pts = (1.20, 0.7, -1.57) # y is estimated 
        center_goal_pts = (0.20, 0.20, -(1.57/2)) # x and y are estimated off of calculated ball placement of (0.15, 0.15)

        match q:
            case 1:
                if self.in_bounds(robo_pose, long_goal_pts):
                    return 12
                elif self.in_bounds(robo_pose, center_goal_pts):
                    return 11
            case 2:
                long_goal_pts = (-long_goal_pts[0], long_goal_pts[1])
                center_goal_pts = (-center_goal_pts[0], center_goal_pts[1])

                if self.in_bounds(robo_pose, long_goal_pts):
                    return 22
                elif self.in_bounds(robo_pose, center_goal_pts):
                    return 21
            case 3:
                long_goal_pts = (-long_goal_pts[0], -long_goal_pts[1])
                center_goal_pts = (-center_goal_pts[0], -center_goal_pts[1])

                if self.in_bounds(robo_pose, long_goal_pts):
                    return 32
                elif self.in_bounds(robo_pose, center_goal_pts):
                    return 31
            case 4:
                long_goal_pts = (long_goal_pts[0], -long_goal_pts[1])
                center_goal_pts = (center_goal_pts[0], -center_goal_pts[1])
               
                if self.in_bounds(robo_pose, long_goal_pts):
                    return 42
                elif self.in_bounds(robo_pose, center_goal_pts):
                    return 41
        return 0
    
def main(args=None):
    rclpy.init(args=args)
    node = Scoring()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
      