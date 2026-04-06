#!/usr/bin/env python3

import rclpy
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import PoseArray, Pose2D
from vex_interfaces.msg import WorldState, GoalState, LoaderState, BallArray

'''
# message to give information to strategy AI model
std_msgs/Header header

# robot pose information (could be float arrays...)
geometry_msgs/Pose2D robot_pose
geometry_msgs/Pose2D opponent_pose

# block locations
vex_interfaces/BallArray blocks
std_msgs/Int64MultiArray robot_intake

# loader states
vex_interfaces/LoaderState loaders # maybe not???

# goal states
vex_interfaces/GoalState goals

# game score (red, blue)
std_msgs/Int64MultiArray score
'''

class StrategyAIBridge(Node):
    def __init__(self):
        super().__init__('sai_bridge')

        # subscribe to score, robot pose, objects, goal states, etc
        self.create_subscription(Int64MultiArray, '/game_score', self.score_cb, 10)
        self.create_subscription(GoalState, '/goals', self.goal_ctrl_zone_cb, 10)
        self.create_subscription(PoseArray,'/otto_pose', self.robot_pose_cb, 10)
        self.create_subscription(Int64MultiArray, '/robot_blocks', self.intake_cb, 10)
        self.create_subscription(BallArray, '/field_objects', self.field_objects_cb, 10)
        self.create_subscription(LoaderState, '/loaders', self.loader_cb, 10)
        self.create_subscription(Int64MultiArray, '/blocks_remaining', self.blocks_left_update_callback, 10)

        # timer for controlling publishing rate
        self.create_timer(1.0, self.update_sai_world_state)

        # publisher for the world state on the /sai_input topic
        self.to_sai = self.create_publisher(WorldState, '/sai_input', 10)

        # initialize blocks left at 12 per color
        self.blocks_left_blue = 12
        self.blocks_left_red = 12

        # globals for storing current state
        self.world_state = WorldState()

    def score_cb(self, msg:Int64MultiArray): 
        self.world_state.score = msg
    
    def goal_ctrl_zone_cb(self, msg:GoalState):
        self.world_state.goals = msg

    def robot_pose_cb(self, msg:PoseArray): 
        '''record the current pose of the robot + reformat '''
        robot_pose = Pose2D()
        robot_pose.x = float(msg.poses[-1].position.x)
        robot_pose.y = float(msg.poses[-1].position.y)

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
        robot_pose.theta = float(euler[2]) # get the yaw (z rotation) value from the returned array

        self.world_state.robot_pose = robot_pose
        
    def intake_cb(self, msg:Int64MultiArray): 
        self.world_state.robot_intake = msg

    def loader_cb(self, msg:Int64MultiArray):
        self.world_state.loaders = msg

    def field_objects_cb(self, msg:BallArray):
        self.world_state.blocks = msg
    
    def blocks_left_update_callback(self, msg:Int64MultiArray):
        self.world_state.blocks_left = msg
    
    def update_sai_world_state(self): 
        # build header msg
        self.world_state.header.stamp = self.get_clock().now().to_msg()
        self.world_state.header.frame_id = 'map'

        # fake opponent pose for now
        opp_pose = Pose2D()
        opp_pose.x, opp_pose.y = float(0.0), float(0.0)
        opp_pose.theta = float(0.0)
        self.world_state.opponent_pose = opp_pose

        # publish new world state
        self.to_sai.publish(self.world_state)


def main(args=None):
    rclpy.init(args=args)
    node = StrategyAIBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()