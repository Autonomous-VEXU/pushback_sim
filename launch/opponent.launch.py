#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
   # pose bridge for opponent
    opponent_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='opponent_bridge',
        arguments=['model/opponent/pose@geometry_msgs/msg/PoseStamped@gz.msgs.Pose'], 
        remappings=[('/model/opponent/pose', '/opponent/pose')]
    )
   
    # opponent model node
    opponent_node = Node(
        package='pushback_sim',
        executable='opponent.py',
        output='screen'
    )

    return LaunchDescription([
      opponent_node, 
      opponent_pose_bridge
    ])