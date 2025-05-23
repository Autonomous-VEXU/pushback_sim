import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    #file paths
    vex_path = os.path.join(get_package_share_directory('vex_pushback'))

    # set gz sim resource path
    gz_sim_resource = setEnvironmentVariable(
        name = 'GZ_SIM_RESOURCE_PATH',
        value = [
            os.path.join(vex_path, 'worlds'), ':' +
            str(Path(vex_path).parent.resolve())
        ]
    )

    # arguments for gz sim... i guess
    args = LaunchDescription([
            DeclareLaunchArgument('world', default_value='vex_pushback', description='sim world'),
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
        args,
        gazebo
    ])