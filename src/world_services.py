#!/usr/bin/env python3
import rclpy
import numpy as np
import random

from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation as R
from pushback_sim.msg import Ball, GoalState
from pushback_sim.srv import OutputBall, Loader, IntakeBall
from typing import Tuple, List
from collections import deque
from dataclasses import dataclass

from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from ros_gz_interfaces.msg import EntityFactory, Entity

@dataclass
class Goal:
    capacity: int
    contents: deque
    endpoints: List
    height: float

class WorldServices(Node):
    def __init__(self):
        super().__init__('world_services')

        # NOTE: red = 1, blue = 2!

        # subscribe to goal contents
        self.create_subscription(GoalState, '/goals', self.save_goal_state, 10) # might upgrade message to "worldState" to include loaders

        # subscribe to otto's location
        self.create_subscription(PoseArray, '/otto_pose', self.robot_pose_callback, 10)

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

        # goals represented as custom data types
        self.goal_1_4 = Goal(
            capacity = 15, # make a param
            endpoints = [12, 42],
            contents = deque(maxlen=16),
            height = 0.39
        )

        self.goal_2_3 = Goal(
            capacity = 15,
            endpoints = [22, 32],
            contents = deque(maxlen=16),
            height = 0.39
        )

        self.center_mid = Goal(
            capacity = 6,
            endpoints = [11, 31],
            contents = deque(maxlen=7),
            height = 0.27
        )

        self.center_low = Goal(
            capacity = 6,
            endpoints = [21, 41],
            contents = deque(maxlen=7),
            height = 0.06
        )

        # locations
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
            1 : [21, 41], # low
            2 : [11, 31], # medium
            3 : [12, 22, 32, 42] # tall
        }

    def save_goal_state(self, msg:GoalState):
        '''saves the world state'''
        self.goal_state = msg
    
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

    # IntakeBall Service callback + functionality
    def intake_ball(self, request, response): 
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

    # OutputBall Service callback + fucntionality 
    def output_ball(self, request, response):
        '''meta function that determines what action should occur with the ball being output based on 
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
        '''handles if the robot has blocks but is not in a scoring location or wrong height is called'''
        drop_x = self.robot_x + random.uniform(-0.3, 0.3) 
        drop_y = self.robot_y + random.uniform(-0.3, 0.3)

        self.spawn_block(ball_color, drop_x, drop_y, 0.4)
        
        self.get_logger().info("dropped ball!")
        
    def score_goal(self, goal_id:int, color:int):
        '''logic that is executed when a goal is scored on'''

        self.clear_goal(goal_id)
        
        match goal_id:
            case 11:
                self.center_mid.contents.appendleft(color)
                self.update_goal_entities(self.center_mid, color, True)
            case 21:
                self.center_low.contents.appendleft(color)
                self.update_goal_entities(self.center_low, color, True)
            case 31:
                self.center_mid.contents.append(color)
                self.update_goal_entities(self.center_mid, color, False)
            case 41:
                self.center_low.contents.append(color)
                self.update_goal_entities(self.center_low, color, False)
            case 12:
                self.goal_1_4.contents.appendleft(color)
                self.update_goal_entities(self.goal_1_4, color, True)
            case 22:
                self.goal_2_3.contents.appendleft(color)
                self.update_goal_entities(self.goal_2_3, color, True)
            case 32:
                self.goal_2_3.contents.append(color)
                self.update_goal_entities(self.goal_2_3, color, False)
            case 42:
                self.goal_1_4.contents.append(color)
                self.update_goal_entities(self.goal_1_4, color, False)
            
    def update_goal_entities(self, goal:Goal, color:int, left:bool):
        '''updates the goal entity that was just scored on'''

        def calc_goal_shift(goal:Goal, input_coords:Tuple[int], left: bool):
            '''helper for calculating the spawn point for each goal when its capacity is reached'''
            x, y, z = input_coords
            tol = 0.08

            if goal.height == 0.39: # long goals
                if left:  y = y + tol # eventually will be a parameter...
                else: y = y - tol
            
            elif goal.height == 0.27: # top center
               if left: x, y = x + tol, y + tol
               else: x, y = x - tol, y - tol

            elif goal.height == 0.06: # lower center
                if left: x, y = x - tol, y + tol
                else: x, y = x + tol, y - tol

            return x, y, z

        if len(goal.contents) == goal.capacity + 1: # max deque capacity is goal capacity + 1 to prevent data loss
            if left:
                dropped_ball = goal.contents.pop() # if scoring left, want to pop right
                x, y, z = self.goal_locations[goal.endpoints[1]] # want the opposite endpoint from the scoring one
                x, y, z = calc_goal_shift(goal, (x, y, z), False)
                self.spawn_block(dropped_ball, x, y, z)
                
            else:
                dropped_ball = goal.contents.popleft()
                x, y, z = self.goal_locations[goal.endpoints[0]]
                x, y, z = calc_goal_shift(goal, (x, y, z), True)
                self.spawn_block(dropped_ball, x, y, z)

        points = self.get_intermediate_points(  
            self.goal_locations[goal.endpoints[0]][:2],  # (x, y) from first endpoint
            self.goal_locations[goal.endpoints[1]][:2],  # (x, y) from second endpoint
            goal.capacity 
        )

        self.get_logger().info(f"Goal contents: {goal.contents}")
        
        for (x,y), color in zip(points, goal.contents):
            self.spawn_block(color, x, y, goal.height)

    def clear_goal(self, goal_id:int):
        '''remove all of the blocks from a goal'''
        if goal_id == 22 or goal_id == 32: # long goal b
            for block in self.goal_state.long_b.object_array:
               self.delete_block(block.id)
            
        elif goal_id == 42 or goal_id == 12: # long goal a
            for block in self.goal_state.long_a.object_array:
               self.delete_block(block.id)

        elif goal_id == 11 or goal_id == 31: # center high
            for block in self.goal_state.center_high.object_array:
               self.delete_block(block.id)

        elif goal_id == 21 or goal_id == 41: # center low
            for block in self.goal_state.center_low.object_array:
               self.delete_block(block.id)
    
    # Loader service functions
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
                response.success = False 
                self.get_logger().info(f"NO RED BLOCKS LEFT!!!")
                return response
            model_pkg_path = '/home/kymadogg/ros2_ws/src/mqp/pushback_sim/models/red-sphere/model.sdf' # make sure to make this a parameter!
            self.red_blocks_left = self.red_blocks_left - 1
            self.get_logger().info(f"Red blocks left: {self.red_blocks_left}")

        if request.color == 2:
            if self.blue_blocks_left == 0:
                response.success = False 
                self.get_logger().info(f"NO BLUE BLOCKS LEFT!!!")
                return response
            model_pkg_path = '/home/kymadogg/ros2_ws/src/mqp/pushback_sim/models/blue-sphere/model.sdf'
            self.blue_blocks_left = self.blue_blocks_left - 1
            self.get_logger().info(f"Blue blocks left: {self.blue_blocks_left}")
        
        else:
            response = False
            return response
        
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

    # Helper functions (entities)
    def spawn_block(self, color:int, x:float, y:float, z:float):
        '''SpawnEntity wrapper function for spawning in a block'''
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

    def delete_block(self, entity_id): # msg = ball id
        '''DeleteEntity wrapper function for deleting a block'''
        ball = Entity()
        ball.id = int(entity_id)  # copy id of picked up ball to entity
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
        '''how i find the locations of each ball'''
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

