#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

def generate_launch_description():
    # file + directory paths
    this_dir = get_package_share_directory('pushback_sim')
    otto_gz = get_package_share_directory('otto_gazebo')
    otto_br = get_package_share_directory('otto_bringup')
    sai_dir = get_package_share_directory('sai')

    # strategy AI bridge launch arg
    teleop_toggle = LaunchConfiguration('teleop')
    teleop_toggle_cmd = DeclareLaunchArgument(
        'teleop',
        default_value='true',
        description='conditionally launches teleop control'
    ) 

    world_ctrl = LaunchConfiguration('world_ctrl')
    world_ctrl_cmd = DeclareLaunchArgument(
        'world_ctrl',
        default_value='true',
        description='toggles simulation post tracking and backend services'
    ) 

    nav2_toggle = LaunchConfiguration('nav2')
    nav2_toggle_cmd = DeclareLaunchArgument(
        'nav2',
        default_value='false',
        description='toggles nav2 mppi controller'
    ) 

    sai_toggle = LaunchConfiguration('sai')
    sai_toggle_cmd = DeclareLaunchArgument(
        'sai',
        default_value='true',
        description='toggles the strategy AI model'
    ) 

    # world launch file
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'launch', 'world_select.launch.py')),
        launch_arguments={'world': 'pushback'}.items()
    )

    # spawn robot
    otto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(otto_gz, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'x_pose': '0.0','y_pose': '-1.0'}.items()
    )

    # enable teleop control
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(otto_br, 'launch', 'controller.launch.py')),
        condition=IfCondition(teleop_toggle)
    )

    # all of the sim backend functions
    sim_backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'launch', 'sim_backend.launch.py')),
        condition=IfCondition(world_ctrl)
    )

    # node that launches the strategy AI model
    sai_node = ExecuteProcess(
        cmd=['/home/kymadogg/ros2_ws/src/mqp/.sai_env/bin/python3', '-m', 'sai.sai_node'],
        condition=IfCondition(sai_toggle)
    )

    # launchfile for basic nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(otto_gz, 'launch', 'nav2.launch.py')),
        condition=IfCondition(nav2_toggle)
    )

    delay_ai = TimerAction(period=10.0, actions=[sai_node])

    return LaunchDescription([
        teleop_toggle_cmd,
        sai_toggle_cmd,
        nav2_toggle_cmd,
        world_ctrl_cmd,
        world,
        delay_ai,
        nav2_launch,
        otto,
        teleop,
        sim_backend
    ])