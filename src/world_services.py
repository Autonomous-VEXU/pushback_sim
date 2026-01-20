#!/usr/bin/env python3
import rclpy
import numpy as np
import os

from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray
from pushback_sim.msg import Ball, BallArray, GoalState
from pushback_sim.srv import OutputBall, Loader, IntakeBall
from typing import Tuple
from collections import deque

from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from ros_gz_interfaces.msg import EntityFactory, Entity

class WorldServices(Node):
    def __init__(self):
        super().__init__('world_services')
        # load params from yaml file
        # self.package_path =  self.get_parameter('pkg_path').get_parameter_value().string_value

        # subscribe to goal contents
        self.create_subscription(GoalState, '/goals', self.save_goal_state, 10) # might upgrade message to "worldState" to include loaders

        # my services
        self.output_ball = self.create_service(OutputBall, '/score_ball', self.output_ball)
        self.add_to_loader = self.create_service(Loader, '/loader', self.add_to_loader)
        self.intake_ball = self.create_service(IntakeBall, '/robot_intake', self.intake_ball)

        # gazebo services
        self.remove_ball = self.create_client(DeleteEntity, '/world/pushback/remove')
        self.spawn_ball = self.create_client(SpawnEntity, '/world/pushback/create')

        # world and robot state variables
        self.robot_intake = []
        self.blue_blocks_left = 12
        self.red_blocks_left = 12

        # my global values
        self.goal_locations = {
            11: (0.15, 0.15, 0.27),
            12: (1.20, 0.57, 0.39),
            21: (-0.15, 0.15, 0.06),
            22: (-1.20, 0.57, 0.39),
            31: (-0.15, -0.15, 0.27),
            32: (-1.20, -0.57, 0.39),
            41: (0.15, -0.15, 0.06),
            42: (1.20, -0.57, 0.39)
        }

        self.loader_locations = {
            13: (1.19, 1.72, 0.5),
            23: (-1.19, 1.72, 0.5),
            33: (-1.19, -1.72, 0.5),
            43: (1.19, -1.72, 0.5)
        }

        self.goal_heights = {
            1 : [], # low
            2 : [], # medium
            3 : [12, 22, 32, 42] # tall
        }

    def save_goal_state(self, msg:GoalState):
        self.goal_state = msg

    # ----------------- IntakeBall Service Functions -------------------- # 
    def intake_ball(self, request, response): # service
        ''' Intake ball service callback function'''
        if len(self.robot_intake) < 12: # if under max capacity
            self.delete_block(request.ball_id)  
            self.robot_intake.append(request.color)
            self.get_logger().info(f'robot intake status: {self.robot_intake}')
            response.success = True
            return response
        else:
            self.get_logger().info("Hopper full")
            self.get_logger().info(f'robot intake status: {self.robot_intake}')
            response.success = False
            return response

    # ----------------- OutputBall Service Functions -------------------- # 
    def output_ball(self, request, response):
        ''' meta function that determines what action should occur with the ball being output based on 
            current location, height attempted, and robot hopper status'''

        if len(self.robot_intake) == 0: # no blocks in hopper edge case
            # log that there is nothing to output
            response.success = False
            return response
        
        ball_color = self.robot_intake.pop(0) # get first item in robot_hopper (i think)
        
        def check_output_height(goal_id, height):
            ''' helper for verifying height matches goal id '''
            if goal_id in self.goal_heights.get(height):
                return True
            elif goal_id == 0 or goal_id not in self.goal_heights.get(height):
                return False
        
        if check_output_height(request.goal_id, request.height): # output height matches goal id
            self.score_goal(request.goal_id, ball_color)
            self.get_logger().info(f'robot intake status: {self.robot_intake}')
            response.success = True
        else: 
            self.drop_ball(ball_color)
            self.get_logger().info(f'robot intake status: {self.robot_intake}')
            response.success = True
        return response

    def drop_ball(self, ball_color:int):
        '''handles if the robot has blocks but is not in a scoring location'''
        self.spawn_block(ball_color, 0.0, 0.0, 0.8)
        self.get_logger().info("dropped ball!")
        
    def score_goal(self, goal_id, color):
        goal_location = self.goal_locations[goal_id]
        
        if goal_id in [12, 42]:  # long goal a
            goal_contents = self.goal_state.long_a.object_array
        elif goal_id in [22, 32]:  # long goal b
            goal_contents = self.goal_state.long_b.object_array
        elif goal_id in [11, 31]:  # center low
            goal_contents = self.goal_state.center_low.object_array
        elif goal_id in [21, 41]:  # center high
            goal_contents = self.goal_state.center_high.object_array
        else:
            self.get_logger().error(f"Invalid goal_id: {goal_id}")
            return
        
        # sort by distance to scoring location (2D)
        def distance_to_goal(ball):
            dx = ball.location.x - goal_location[0]
            dy = ball.location.y - goal_location[1]
            return np.sqrt(dx**2 + dy**2)
        
        goal_contents_sorted = sorted(goal_contents, key=distance_to_goal)

        goal_queue = deque(goal_contents_sorted)

    def clear_goal(self, goal_id:int):
        '''remove all of the blocks from a goal'''
        if goal_id == 22 or goal_id == 32: # long goal b
            for block in self.goal_state.long_b.object_array:
               self.delete_block(block)
        elif goal_id == 42 or goal_id == 12: # long goal a
            for block in self.goal_state.long_a.object_array:
               self.delete_block(block)
        elif goal_id == 11 or goal_id == 31: # center low
            for block in self.goal_state.center_low.object_array:
               self.delete_block(block)
        elif goal_id == 21 or goal_id == 41: # center high
            for block in self.goal_state.center_high.object_array:
               self.delete_block(block)
    
    # ----------------- Loader Service Functions -------------------- # 
    def add_to_loader(self, request:Loader.Request, response):
        '''spawn ball slightly above loader'''

        # if len(loader.id.object_array) >= self.loader_capacity: <-- add in error handling later
        # response.success = False
        # print (loader {loader.id} full!)
        # return response

        ball = EntityFactory()
        ball.allow_renaming = True

        if request.color == 1:
            if self.red_blocks_left == 0:
                response.success = False # maybe add log statement to say there are no blocks left?
                return response
            model_pkg_path = '/home/kymadogg/ros2_ws/src/mqp/pushback_sim/models/red-sphere/model.sdf'
            self.red_blocks_left = self.red_blocks_left - 1

        if request.color == 2:
            if self.blue_blocks_left == 0:
                response.success = False # maybe add log statement to say there are no blocks left?
                return response
            model_pkg_path = '/home/kymadogg/ros2_ws/src/mqp/pushback_sim/models/blue-sphere/model.sdf'
            self.blue_blocks_left = self.blue_blocks_left - 1
        
        # full_path = os.path.join(self.package_path, model_pkg_path)
        ball.sdf_filename = model_pkg_path
        loader_pose = self.loader_locations[request.loader_id]
        ball.pose.position.x = loader_pose[0]
        ball.pose.position.y = loader_pose[1]
        ball.pose.position.z = loader_pose[2]

        loader_req = SpawnEntity.Request()
        loader_req.entity_factory = ball
        
        self.spawn_ball.call_async(loader_req)

        self.get_logger().info(f"added block to loader {request.loader_id}")
        response.success = True
        return response

    # ----------------- Helper Functions -------------------- # 

    def spawn_block(self, color:int, x:float, y:float, z:float):
        ball = EntityFactory()
        ball.allow_renaming = True

        if color == 1:
            model_pkg_path = '/home/kymadogg/ros2_ws/src/mqp/pushback_sim/models/red-sphere/model.sdf'

        elif color == 2:
            model_pkg_path = '/home/kymadogg/ros2_ws/src/mqp/pushback_sim/models/blue-sphere/model.sdf'
        
        ball.sdf_filename = model_pkg_path
        ball.pose.position.x = x
        ball.pose.position.y = y
        ball.pose.position.z = z

        spawn_req = SpawnEntity.Request()
        spawn_req.entity_factory = ball
        
        self.spawn_ball.call_async(spawn_req)
        return True

    def delete_block(self, msg): # msg = ball id
        '''DeleteEntity wrapper function for deleting a block'''
        ball = Entity()
        ball.id = msg # copy id of picked up ball to entity
        delete_req = DeleteEntity.Request()
        delete_req.entity = ball
        self.remove_ball.call_async(delete_req)
    
    def ball_color(self, ball:Ball):
        '''return ball color. red = 1, blue = 2'''
        red_identifyers = ["red", "R"]
        if any(sub in ball.object_name for sub in red_identifyers):
            return 1
        else:
            return 2

    def get_intermediate_points(self, p1, p2, num_points):
        if num_points < 2:
            return [p1, p2] if p1 != p2 else [p1]
            
        x1, y1 = p1
        x2, y2 = p2
        
        x_coords = np.linspace(x1, x2, num_points)
        y_coords = np.linspace(y1, y2, num_points)
        
        points = list(zip(x_coords, y_coords))
        return points
    
def main(args=None):
    rclpy.init(args=args)
    node = WorldServices()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

