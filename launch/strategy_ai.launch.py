#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    '''launches nodes needed for integrating with the strategy AI model'''

    delay = LaunchConfiguration('ai_delay')
    delay_cmd = DeclareLaunchArgument(
        'ai_delay',
        default_value='10.0',
        description='amount of time to delay launching the strategy AI model'
    ) 

    # strategy AI bridge
    sai_bridge = Node(
        package='pushback_sim',
        executable='strategy_ai_bridge.py',
        output='screen'
    )

    # node that launches the strategy AI model
    sai_node = ExecuteProcess(
        cmd=['/home/kymadogg/ros2_ws/src/mqp/.sai_env/bin/python3', '-m', 'sai.sai_node']
    )

    # frames be wrong
    sai_map_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_node',
        arguments=['-1.75', '1.75', '0.2', '4.71239', '0', '0', 'map', 'sai_map']
    )

    delayed_sai_node = TimerAction( 
        period=delay, 
        actions=[sai_node]
    )

    return LaunchDescription([
        delay_cmd,
        sai_map_frame,
        sai_bridge,
        delayed_sai_node
    ])