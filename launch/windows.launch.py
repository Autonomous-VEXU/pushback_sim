#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
   
    # K: what does this file even achieve if you can't set the world?
    world = os.path.join(
        get_package_share_directory('pushback_sim'),
        'worlds',
        'pushback.sdf'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

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

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gz_sim_resource)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld
