from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Packages
    pkg_goddard_description = get_package_share_directory('goddard_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Nodes
    rviz = Node(package='rviz2',
                executable='rviz2',
                arguments=[
                    '-d',
                    os.path.join(pkg_goddard_description, 'rviz', 'goddard_gazebo.rviz')
                ],
                parameters=[{
                    'use_sim_time': use_sim_time
                }])

    return LaunchDescription([ 
        # Launch arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        
        # Nodes
        rviz,
    ])
    

