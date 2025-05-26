import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # base file path for the package
    vex_path = os.path.join(get_package_share_directory('vex_pushback'))

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
            DeclareLaunchArgument('world', default_value='pushback_comp', description='sim world'),
    ])

    # actually run gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments = [
            ('gz_args', [LaunchConfiguration('world'),'.sdf',' -v 4',' -r'])
        ]
    )
    # launch each item defined above by returning the variable
    return LaunchDescription([
        gz_sim_resource,
        arguments,
        gazebo,
    ])