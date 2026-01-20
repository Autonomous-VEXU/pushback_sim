#!/usr/bin/env python3
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseArray, Point
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray
from pushback_sim.msg import Ball, BallArray, GoalState
from typing import Tuple
from pushback_sim.srv import IntakeBall

from ros_gz_interfaces.srv import DeleteEntity
from ros_gz_interfaces.msg import Entity

'''
things to note:
 - when you are not in a goal zone and you hit the score button, a ball will drop on the ground == drop ball service
 - in a goal zone == score ball service
 - 
'''

class FieldLocation(Node):
    def __init__(self):
        super().__init__('field_location')
        self.create_subscription(Joy, '/joy', self.controller_callback, 10)
        self.create_subscription(BallArray, '/object_locations', self.object_location_callback, 10)

        # debug topic that i should remove later
        self.debug = self.create_publisher(Float64MultiArray, '/debug', 10)

        # publishers for entities in goals
        self.goals = self.create_publisher(GoalState, '/goals', 10)

        # robot location / pose subscriber
        self.create_subscription(PoseArray, '/otto_pose', self.robot_pose_callback, 10)

        # service client to remove a ball when it is picked up
        self.remove_ball = self.create_client(DeleteEntity, '/world/pushback/remove')

        # service client for intaking a ball
        self.intake_ball = self.create_client(IntakeBall, '/robot_intake')

        self.tol = (0.2, 0.2, 0.1)
        self.z_tol = 0.01

    def controller_callback(self, msg:Joy):
        '''handle controller input'''
        # self.get_logger().info('getting controller messages')
        # check controller input for the intake / scoring buttons being pressed
        if msg.buttons[0] == 1: # ball out
            location = self.check_location()
            self.get_logger().info(f'Otto is at goal ID: {location}')
        elif msg.buttons[1] == 1: # activate intake
            # intake funtion
            self.check_collision()

    def robot_pose_callback(self, msg:PoseArray):
        '''record the current pose of the robot'''
        self.robot_x = msg.poses[-1].position.x
        self.robot_y = msg.poses[-1].position.y

        quat = msg.poses[-1].orientation
        quat_array = np.array([quat.x, quat.y, quat.z, quat.w])

        # normalize
        quat_norm = np.linalg.norm(quat_array)
        if quat_norm > 0: 
            quat_normalized = quat_array / quat_norm
        else:
            quat_normalized = quat_array 
            
        rotation = R.from_quat(quat_normalized)

        euler = rotation.as_euler('xyz') # THIS IS IN RADIANS!
        self.robot_r = euler[2] # get the yaw (z rotation) value from the returned array
    
    def object_location_callback(self, msg:BallArray):
        '''read all of the objects published from the pose_bridge node and see if they are in a goal or not '''
        self.objects=msg

        goal_state = GoalState()
        long_1_4 = []
        long_2_3 = []
        center_low = []
        center_high = []

        def dist_from_line(p1:tuple, p2:tuple, p3:tuple):
            ''' distance a ball is from the line formed by the two ends of the goal'''
            p1 = np.array(p1)
            p2 = np.array(p2)
            p3 = np.array(p3)
           
            p1p2_vec = p2 - p1
            p1p3_vec = p3 - p1
            
            cross_product_mag = np.linalg.norm(np.cross(p1p2_vec, p1p3_vec), axis=-1) if p3.ndim > 1 else np.linalg.norm(np.cross(p1p2_vec, p1p3_vec))
            
            p1p2_mag = np.linalg.norm(p1p2_vec)
            
            if p1p2_mag == 0:
                return np.linalg.norm(p1p3_vec, axis=-1) if p3.ndim > 1 else np.linalg.norm(p1p3_vec)

            return cross_product_mag / p1p2_mag

        # sort the ball array into the different goals. there is probably a way better way to do this that I am not doing. 
        # TODO: refactor all of this later.

        for ball in msg.object_array: 
            ball_xy = (ball.location.x, ball.location.y)
            if ball.location.z > 0.38: # long goals
                if self.in_bounds(ball_xy, (1.20, 0), (0.04, 0.6)): # if in goal_1_4
                    long_1_4.append(ball)
                elif self.in_bounds(ball_xy, (-1.20, 0), (0.04, 0.6)): # if in goal_2_3
                    long_2_3.append(ball)
            elif self.in_bounds(ball_xy, (0.0, 0.0), (0.15, 0.15)): # center goals
                if 0.25 < ball.location.z < 0.30 and dist_from_line((0.15, 0.15), (-0.15, -0.15), ball_xy) < 0.03: # top center goal
                    center_high.append(ball)
                elif 0.05 < ball.location.z < 0.10 and dist_from_line((-0.15, 0.15), (0.15, -0.15), ball_xy) < 0.03: # lower center goal
                    center_low.append(ball)
            else:
                continue

        # publish the updated GoalState message
        goal_state.center_low = BallArray()
        goal_state.center_low.object_array = center_low
        
        goal_state.center_high = BallArray()
        goal_state.center_high.object_array = center_high
        
        goal_state.long_a = BallArray()
        goal_state.long_a.object_array = long_1_4
        
        goal_state.long_b = BallArray()
        goal_state.long_b.object_array = long_2_3

        self.goals.publish(goal_state)
            
    def check_collision(self):
        '''did the robot collide with a ball or not'''
        h, k = self.robot_x, self.robot_y
        th = self.robot_r
        
        offset = 0.15 
        
        x = h + offset * np.cos(th)
        y = k + offset * np.sin(th)

        db = Float64MultiArray()
        db.data = [x, y]
        
        self.debug.publish(db) # remove this later

        # validate ball location helper method
        def in_intake_zone(ref_x, ref_y, ball_pos: Point):
            z_t = 0.06
            xy_t = 0.10

            dx = ball_pos.x - ref_x
            dy = ball_pos.y - ref_y

            th = self.robot_r

            # world → robot frame
            dx_r =  np.cos(th) * dx + np.sin(th) * dy
            dy_r = -np.sin(th) * dx + np.cos(th) * dy

            return (
                abs(dx_r) < xy_t and
                abs(dy_r) < xy_t and
                ball_pos.z < z_t
            )
        
        # check all objects for a collision
        for ball in self.objects.object_array:
            if in_intake_zone(x, y, ball.location):

                # call the intake service --> intake.py
                intake_req = IntakeBall.Request()
                intake_req.ball_id = ball.id
                self.intake_ball.call_async(intake_req)
                return
            
        self.get_logger().info("no ball found")
        
    def in_bounds(self, pose:Tuple, goal:Tuple, tol:tuple):
        '''helper method so i dont have to type the same thing 20 times '''
        error_x = abs(pose[0] - goal[0])
        error_y = abs(pose[1] - goal[1])
       # error_r = abs(pose[2] - goal[2])

        if error_x < tol[0] and error_y < tol[1]: # add rotational error back soon ...
            return True
        else:
            return False
    
    def get_quadrant(self):
        '''which quadrant of the field am i in'''
        if self.robot_x >= 0 and self.robot_y >= 0: # quadrant 1
            return 1
        elif self.robot_x < 0 and self.robot_y >= 0: # quadrant 2
            return 2
        elif self.robot_x < 0 and self.robot_y < 0: # quadrant 3
            return 3
        elif self.robot_x >= 0 and self.robot_y < 0: # quadrant 4
            return 4
        
    def check_location(self):
        '''verify that I am in a location that I can score'''
        robot_pose = (self.robot_x, self.robot_y)
        q = self.get_quadrant()

        long_goal_pts = (1.20, 0.7, -1.57) # y is estimated 
        center_goal_pts = (0.20, 0.20, -(1.57/2)) # x and y are estimated off of calculated ball placement of (0.15, 0.15)

        match q:
            case 1:
                if self.in_bounds(robot_pose, long_goal_pts, self.tol):
                    return 12
                elif self.in_bounds(robot_pose, center_goal_pts,self.tol):
                    return 11
            case 2:
                long_goal_pts = (-long_goal_pts[0], long_goal_pts[1])
                center_goal_pts = (-center_goal_pts[0], center_goal_pts[1])

                if self.in_bounds(robot_pose, long_goal_pts,self.tol):
                    return 22
                elif self.in_bounds(robot_pose, center_goal_pts,self.tol):
                    return 21
            case 3:
                long_goal_pts = (-long_goal_pts[0], -long_goal_pts[1])
                center_goal_pts = (-center_goal_pts[0], -center_goal_pts[1])

                if self.in_bounds(robot_pose, long_goal_pts, self.tol):
                    return 32
                elif self.in_bounds(robot_pose, center_goal_pts, self.tol):
                    return 31
            case 4:
                long_goal_pts = (long_goal_pts[0], -long_goal_pts[1])
                center_goal_pts = (center_goal_pts[0], -center_goal_pts[1])
               
                if self.in_bounds(robot_pose, long_goal_pts, self.tol):
                    return 42
                elif self.in_bounds(robot_pose, center_goal_pts, self.tol):
                    return 41
        return 0
    
    def remove_ball_from_world(self, msg:Ball):
        '''remove this soon'''
        ball = Entity()
        ball.id = msg.id # copy id of picked up ball to entity

        delete_req = DeleteEntity.Request()
        delete_req.entity = ball

        self.remove_ball.call_async(delete_req)
    
def main(args=None):
    rclpy.init(args=args)
    node = FieldLocation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
