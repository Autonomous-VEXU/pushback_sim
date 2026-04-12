import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ros_gz_bridge.actions import RosGzBridge

def generate_launch_description():
    # file + directory paths
    # this_dir = get_package_share_directory('pushback_sim')
    # otto_gz = get_package_share_directory('otto_gazebo')
    # otto_br = get_package_share_directory('otto_bringup')

    # opponent toggle
    opponent_launch = LaunchConfiguration('opponent')
    opponent_launch_cmd = DeclareLaunchArgument(
        'opponent',
        default_value='false',
        description='toggles opponent spawning into world + other nodes launching'
    ) 

    # bridge services
    gz_services_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='entity_services_bridge',
        arguments=[
            '/world/pushback/remove@ros_gz_interfaces/srv/DeleteEntity',
            '/world/pushback/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/pushback/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    ) 

    # robot model bridges
    otto_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='otto_bridge',
        arguments=['model/Otto/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'], 
        remappings=[('/model/Otto/pose', '/otto_pose')]
    )

    opponent_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='opponent_bridge',
        arguments=['model/opponent/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'], 
        remappings=[('/model/opponent/pose', '/opponent/pose')],
        condition=IfCondition(opponent_launch)
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

    # opponent model node
    opponent_node = Node(
        package='pushback_sim',
        executable='opponent.py',
        output='screen',
        condition=IfCondition(opponent_launch)
    )

    return LaunchDescription([
        opponent_launch_cmd, 
        otto_pose_bridge,
        opponent_pose_bridge,
        object_poses,
        gz_services_bridge,
        scoring,
        opponent_node,
        world_services,
        locator
    ])
