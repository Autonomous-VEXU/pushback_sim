import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # file + directory paths
    this_dir = get_package_share_directory('pushback_sim')
    otto_gz = get_package_share_directory('otto_gazebo')
    otto_br = get_package_share_directory('otto_bringup')
   
    # world launch file
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_dir, 'launch', 'world_select.launch.py')),
        launch_arguments={'world': 'pushback'}.items()
    )

    # spawn robot
    otto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(otto_gz, 'launch', 'spawn_robot.launch.py')),
        launch_arguments={'x_pose': '0.5','y_pose': '0.0'}.items()
    )

    # enable teleop control
    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(otto_br, 'launch', 'controller.launch.py'))
    )

    # bridge services
    delete_object_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='entity_services_bridge',
        arguments=[
            '/world/pushback/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/pushback/create@ros_gz_interfaces/srv/SpawnEntity'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    ) 

    # pose bridge
    object_poses = Node(
        package='pushback_sim',
        executable='pose_bridge.py'
        # output='screen'
    )

    # scoring
    scoring = Node(
        package='pushback_sim',
        executable='field_location.py',
        output='screen'
    )

    # scoring
    world_services = Node(
        package='pushback_sim',
        executable='world_services.py',
        output='screen'
    )

    return LaunchDescription([
        world, 
        otto, 
        teleop,
        object_poses,
        delete_object_bridge,
        world_services,
        scoring
    ])
