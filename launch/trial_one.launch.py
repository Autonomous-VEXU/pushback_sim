import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('vex_pushback').find('vex_pushback')
    world = os.path.join(pkg_share, 'worlds', 'pushback_v2.sdf')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world,
        description='Gazebo world file'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    return LaunchDescription([
        declare_world_cmd,
        gazebo_launch
    ])