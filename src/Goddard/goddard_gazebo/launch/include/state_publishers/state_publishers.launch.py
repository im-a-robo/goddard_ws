from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

import os
import xacro

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Packages
    pkg_goddard_description = get_package_share_directory('goddard_description')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_joint_state_publisher_gui = LaunchConfiguration('use_joint_state_publisher_gui', default='true')

    # Get urdf path a run xacro command to read the urdf
    urdf_path = os.path.join(pkg_goddard_description, 'urdf', 'example_robot.urdf.xacro')
    robot_desc = xacro.process_file(urdf_path).toxml()

    # Nodes
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='screen',
                                 parameters=[{
                                     'use_sim_time': use_sim_time,
                                     'robot_description': robot_desc
                                 }])

    join_state_publisher = Node(package='joint_state_publisher',
                                executable='joint_state_publisher',
                                output='screen',
                                condition=UnlessCondition(
                                    use_joint_state_publisher_gui
                                ))
    
    join_state_publisher_gui = Node(package='joint_state_publisher_gui',
                                executable='joint_state_publisher_gui',
                                output='screen',
                                condition=IfCondition(
                                        use_joint_state_publisher_gui
                                ))
    
    return LaunchDescription([   
        # Launch Arguments
        DeclareLaunchArgument('use_sim_time',
                              default_value='true',
                              description='Use simulation clock if true'),
        DeclareLaunchArgument('use_joint_state_publisher_gui', 
                              default_value='true',
                              description='Use the joint state publisher gui when true'),        
        
        # Nodes
        robot_state_publisher,
        join_state_publisher,
        join_state_publisher_gui
    ])
    

