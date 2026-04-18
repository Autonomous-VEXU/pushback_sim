from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

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
        #output='screen'
    )

    # world services endpoint
    world_services = Node(
        package='pushback_sim',
        executable='world_services.py',
        #output='screen'
    )

    # scoring the game
    scoring = Node(
        package='pushback_sim',
        executable='scoring.py',
        output='screen'
    )

    return LaunchDescription([
        otto_pose_bridge,
        object_poses,
        gz_services_bridge,
        scoring,
        world_services,
        locator
    ])
