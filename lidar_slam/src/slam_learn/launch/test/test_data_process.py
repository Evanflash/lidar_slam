import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import expanduser


def generate_launch_description():

 # Nodes Configurations
  config_file = os.path.join(get_package_share_directory('slam_learn'), 'config', 'config.yaml')
  rviz_config = os.path.join(get_package_share_directory('slam_learn'), 'rviz/test','test_data_process.rviz')

  #######################
  # data-publisher 
  data_config_file = '/home/evan/code/ndt-loc/src/data-publisher/config/data_config.yaml'

  #######################
  data_process_node = Node(
    package='slam_learn',
    executable='data_process_node',
    output='screen',
    parameters=[config_file],
  )

  front_end_node = Node(
    package='slam_learn',
    executable='front_end_node',
    output='screen',
    parameters=[config_file]
  )

  back_end_node = Node(
    package='slam_learn',
    executable='back_end_node',
    output='screen',
  )

  # Rviz
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config],
    output='screen'
  )

  data = Node(
    package='data-publisher',
    executable='lidar_data_node',
    name='data',
    parameters=[data_config_file]
  )
  # test
  test_data_process = Node(
    package='slam_learn',
    executable='data_process_test',
    output='screen'
  )

  ld = LaunchDescription()

  ld.add_action(data)
  ld.add_action(data_process_node)
  ld.add_action(front_end_node)
  # ld.add_action(back_end_node)
  ld.add_action(rviz_node)
  ld.add_action(test_data_process)

  return ld
