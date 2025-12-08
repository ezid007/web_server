#!/usr/bin/env python3
#
# Navigation2 + Rosbridge Launch File for Robotis Package
# Map path: /home/tuf/web_server/map/map.yaml
# 웹 대시보드에서 Nav2 Goal 설정을 위해 rosbridge 포함

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Map file path
    map_file = '/home/tuf/web_server/map/map.yaml'
    
    # Get turtlebot3_navigation2 package share directory
    nav2_launch_dir = os.path.join(
        FindPackageShare('turtlebot3_navigation2').find('turtlebot3_navigation2'),
        'launch'
    )
    
    # Include navigation2 launch file with custom map
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation2.launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false'
        }.items()
    )
    
    # Rosbridge websocket node for web dashboard Nav2 goal
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '',
            'retry_startup_delay': 5.0,
        }]
    )
    
    ld = LaunchDescription()
    ld.add_action(navigation2_cmd)
    ld.add_action(rosbridge_node)
    
    return ld


