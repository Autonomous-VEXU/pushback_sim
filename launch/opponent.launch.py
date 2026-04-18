#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

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