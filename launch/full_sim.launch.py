import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # file + directory paths
    this_dir = get_package_share_directory('pushback_sim')
    otto_gz = get_package_share_directory('otto_gazebo')
    otto_br = get_package_share_directory('otto_bringup')

    # strategy AI bridge launch arg
    sai = LaunchConfiguration('s_ai')
    sai_cmd = DeclareLaunchArgument(
        's_ai',
        default_value='false',
        description='conditionally launches the strategy AI bridge node'
    ) 

    # strategy AI bridge launch arg
    teleop_toggle = LaunchConfiguration('teleop')
    teleop_toggle_cmd = DeclareLaunchArgument(
        'teleop',
        default_value='true',
        description='conditionally launches teleop control'
    ) 
   
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
        PythonLaunchDescriptionSource(os.path.join(otto_br, 'launch', 'controller.launch.py')),
        condition=IfCondition(teleop_toggle)
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

    # field locations
    locator = Node(
        package='pushback_sim',
        executable='field_location.py',
        output='screen'
    )

    # world services endpoint
    world_services = Node(
        package='pushback_sim',
        executable='world_services.py',
        output='screen'
    )

    # scoring the game
    scoring = Node(
        package='pushback_sim',
        executable='scoring.py',
        output='screen'
    )

    # strategy AI bridge
    sai_node = Node(
        package='pushback_sim',
        executable='strategy_ai_bridge.py',
        output='screen',
        condition=IfCondition(sai)
    )

    return LaunchDescription([
        teleop_toggle_cmd,
        world,
        sai_cmd, 
        otto, 
        teleop,
        object_poses,
        delete_object_bridge,
        scoring,
        world_services,
        locator,
        sai_node
    ])
