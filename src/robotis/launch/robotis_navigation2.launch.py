#!/usr/bin/env python3
#
# Navigation2 Launch File for Robotis Package
# Map path: /home/tuf/web_server/map/map.yaml

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
    
    ld = LaunchDescription()
    ld.add_action(navigation2_cmd)
    
    return ld
