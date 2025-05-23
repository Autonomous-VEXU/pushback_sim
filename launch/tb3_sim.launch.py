
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription # type: ignore
from launch.conditions import IfCondition, UnlessCondition # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource # type: ignore
from launch.substitutions import Command, LaunchConfiguration, PythonExpression # type: ignore
from launch_ros.actions import Node # type: ignore
from launch_ros.substitutions import FindPackageShare # type: ignore
 
def generate_launch_description():

  # Set the path to the Gazebo ROS package
  ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')   
   
  # Set the path to vex package
  pkg_share = FindPackageShare(package='vex_pushback').find('vex_pushback')
 
  # world file path
  world_file_name = 'pushback_v2.sdf'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
   
  # Set the path to the SDF model files
  gazebo_models_path = os.path.join(pkg_share, 'vex_pushback')
  os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
 
  ########### YOU DO NOT NEED TO CHANGE ANYTHING BELOW THIS LINE ##############  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
 
  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
     
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')
 
  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')
 
  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
  
  # Specify the actions
   
  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 
  # Create the launch description and populate
  ld = LaunchDescription()
 
  # Declare the launch options
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)
 
  # Add any actions
  #ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
 
  return ld