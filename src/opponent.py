#!/usr/bin/env python3

import rclpy
import numpy as np
import random

from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from geometry_msgs.msg import PoseArray
from scipy.spatial.transform import Rotation as R

class Opponent(Node):
    def __init__(self):
        super().__init__('opponent')

        # subscribe to gazebo topic for opponent pose
        self.create_subscription(PoseArray, '/opponent/pose', self.get_opponent_pose, 10)

        # determine if opponent moves every 2s
        self.create_timer(2.0, self.dumb_behavior)

        # move pose service
        self.move_pose = self.create_client(SetEntityPose, '/world/pushback/set_pose')
        self.init_opponent = self.create_client(SpawnEntity, '/world/pushback/create')

        self.opp_x = None
        self.opp_y = None
        self.opp_th = None

        self.current_pose_key = 8
        
        self.height = 0.2

    def get_opponent_pose(self, msg:PoseArray):
        '''get the opponent robot pose'''

        self.opp_x = msg.poses[-1].position.x
        self.opp_y = msg.poses[-1].position.y

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
        self.opp_th = euler[2] # get the yaw (z rotation) value from the returned array

        
    @staticmethod
    def us_metric(value:float, unit:str):
        '''takes in either ft or meters and outputs the other'''
        if unit == 'm':
            return value * 0.3048
        elif unit == 'ft':
            return value / 0.3048
        else: 
            ValueError.args(unit)

    def dumb_behavior(self):
        '''move randomly along the pose graph given each time the timer is called'''

        if np.random.binomial(n=1, p=0.7) == 0: # 70% chance of movement
            return  

        moveset = [(-1.24, 1.0), 
                   (-0.718, 0.98), 
                   (0.615, 1.0), 
                   (1.2, 1.03), 
                   (1.55, 0.187), 
                   (0.73, 0.12), 
                   (-0.57, 0.76), 
                   (-1.52, 0.1), 
                   (0.0, 1.0)] 
        
        move_graph = {
            0: [1, 7],
            1: [6, 8, 0],
            2: [3, 5, 8],
            3: [4, 2],
            4: [3],
            5: [2],
            6: [1],
            7: [0],
            8: [1, 2]}
        
        potential_moves = move_graph[self.current_pose_key]
        new_pose_index = random.choice(potential_moves)
        new_pose = moveset[new_pose_index]
        self.teleport_to_pose(new_pose)
        self.current_pose_key = new_pose_index

    def teleport_to_pose(self, pose:tuple):
        '''teleport the opponent model somewhere'''

        move_req = SetEntityPose.Request()

        move_req.entity.name = 'opponent'
        move_req.pose.position.x = pose[0]
        move_req.pose.position.y = pose[1]
        move_req.pose.position.z = self.height

        self.move_pose.call_async(move_req)

def main(args=None):
    rclpy.init(args=args)
    node = Opponent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()