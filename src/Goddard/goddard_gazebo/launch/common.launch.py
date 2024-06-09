from launch_ros.actions import Node

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration

import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ROS packages
    pkg_goddard_gazebo = get_package_share_directory('goddard_gazebo')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_join_state_publisher_gui = LaunchConfiguration('use_join_state_publisher_gui', default='true')
    
    # Launch Descriptions
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_goddard_gazebo, 'launch', 'include', 'rviz', 'rviz.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time
            }.items(),
        condition=IfCondition(use_rviz)
        )
    
    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_goddard_gazebo, 'launch', 'include', 'state_publishers', 'state_publishers.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_join_state_publisher_gui': use_join_state_publisher_gui
        }.items()
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('use_rviz',
                              default_value='true',
                              description='Use rviz when true'),
        DeclareLaunchArgument('use_join_state_publisher_gui', 
                              default_value='true',
                              description='Use the joint state publisher gui when true'),    
        
        rviz,
        state_publishers
        
])
    

