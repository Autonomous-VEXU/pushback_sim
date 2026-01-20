#!/usr/bin/env python3
import rclpy
import numpy as np
import os

from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray
from pushback_sim.msg import Ball, BallArray
from pushback_sim.srv import Score
from typing import Tuple


from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from ros_gz_interfaces.msg import Entity, EntityWrench, EntityFactory

class WorldServices(Node):
    def __init__(self):
        # service server to score a ball

        self.package_path =  self.get_parameter('pkg_path').get_parameter_value().string_value

        self.score = self.create_service(Score, '/score_ball', self.score_ball_shift)
        # self.add_to_loader = self.create_service(AddBlockLoader, '/world/default/create')

        # gazebo services
        self.remove_ball = self.create_client(DeleteEntity, '/world/pushback/remove')
        self.spawn_ball = self.create_client(SpawnEntity, '/world/pushback/create')

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

    def score_ball_shift(self, request):
        if request.goal_id % 10 == 2:
            goal_capacity = 16
        elif request.goal_id % 10 == 1:
            goal_capacity = 6
        

    def clear_goal():
        '''remove all of the blocks from a goal'''

        pass
     
    def add_to_loader(self, loader_id:int, color:int):
        '''spawn ball slightly above loader'''
        ball = EntityFactory()
        ball.allow_renaming = True

        if color == 1:
            model_pkg_path = '/pushback_sim/models/red-sphere/model.sdf'

        elif color == 2:
            model_pkg_path = '/pushback_sim/models/blue-sphere/model.sdf'
        
        full_path = os.path.join(self.package_path, model_pkg_path)
        ball.sdf_filename = full_path
        loader_pose = self.loader_locations[loader_id]
        ball.pose.position.x = loader_pose[0]
        ball.pose.position.y = loader_pose[1]
        ball.pose.position.z = loader_pose[2]

        loader_req = SpawnEntity.Request()
        
        self.spawn_ball.call_async(loader_req)

        self.get_logger("loader_update").info(f"added block to loader {loader_id}")
