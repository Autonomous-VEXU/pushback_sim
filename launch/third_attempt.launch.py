import os
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression 
from launch_ros.substitutions import FindPackageShare 

def generate_launch_description():
    # arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    #file paths
    vex_path = os.path.join(get_package_share_directory('vex_pushback'))

    # set gz sim resource path
    gz_sim_resource = SetEnvironmentVariable(
        name = 'GZ_SIM_RESOURCE_PATH',
        value = [
            os.path.join(vex_path, 'worlds'), ':' +
            str(Path(vex_path).parent.resolve())
        ]
    )

    # arguments for gz sim... i guess
    arguments = LaunchDescription([
            DeclareLaunchArgument('world', default_value='pushback_v2', description='sim world'),
    ])

    # actually run gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments = [
            ('gz_args', [LaunchConfiguration('world'),'.sdf',' -v 4',' -r'])
        ]
    )

    return LaunchDescription([
        gz_sim_resource,
        arguments,
        gazebo
    ])