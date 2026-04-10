import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # base file path for the package
    vex_path = os.path.join(get_package_share_directory('pushback_sim'))

    # secondary file paths for locating resources
    models_path = os.path.join(vex_path, 'models')
    worlds_path = os.path.join(vex_path, 'worlds')

    def get_available_worlds():
        try:
            world_list = [f[:-4] for f in os.listdir(worlds_path) if f.endswith('.sdf')]
            if not world_list:
                return "directory empty"
            return "Gz Sim Worlds: " + ", ".join(world_list)
        except Exception as e:
            return f"ERROR: {e}"
        
    worlds = get_available_worlds()

    # set gz sim resource path
    gz_sim_resource = SetEnvironmentVariable(
        name = 'GZ_SIM_RESOURCE_PATH',
        value=f"{models_path}:{worlds_path}:{vex_path}"
    )

    # arguments for gz sim
    arguments = LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty', description=worlds),
        DeclareLaunchArgument('headless', description="Run Gazebo headless"),
    ])

    # actually run gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments = [
            ('gz_args', [LaunchConfiguration('world'),'.sdf',' -v 4', (' -s' if LaunchConfiguration('headless') else ''),' -r'])
        ]
    )
    # launch each item defined above by returning the variable
    return LaunchDescription([
        gz_sim_resource,
        arguments,
        gazebo
    ])