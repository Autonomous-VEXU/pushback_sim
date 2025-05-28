import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # base file path for the package
    vex_path = os.path.join(get_package_share_directory('pushback_sim'))

    # secondary file paths for locating resources
    models_path = os.path.join(vex_path, 'models')
    worlds_path = os.path.join(vex_path, 'worlds')

    # set gz sim resource path
    gz_sim_resource = SetEnvironmentVariable(
        name = 'GZ_SIM_RESOURCE_PATH',
        value=f"{models_path}:{worlds_path}:{vex_path}"
    )

    # arguments for gz sim
    arguments = LaunchDescription([
        # set world for gazebo
        DeclareLaunchArgument('world', default_value='pushback_v2', description='sim world'),
        # set x position of turtlebot3
        DeclareLaunchArgument('x_pose', default_value='.5', description='turtlebot X coord'),
        # set y position of turtlebot3
        DeclareLaunchArgument('y_pose', default_value='1', description='turtlebot Y coord'),
        # set z position of turtlebot3
        DeclareLaunchArgument('z_pose', default_value='.04', description='turtlebot Z coord'),
    ])

    # actually run gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments = [
            ('gz_args', [LaunchConfiguration('world'),'.sdf',' -v 4',' -r'])
        ]
    )

    #spawn in turtlebot 3
    turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'launch'), '/spawn_turtlebot3.launch.py']),
        launch_arguments = [
            ('x_pose',LaunchConfiguration('x_pose')),
            ('y_pose', LaunchConfiguration('y_pose')),
            ('z_pose', LaunchConfiguration('z_pose'))
        ]
    )

    # launch each item defined above by returning the variable
    return LaunchDescription([
        gz_sim_resource,
        arguments,
        gazebo,
        turtlebot,
    ])