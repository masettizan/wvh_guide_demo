#! /usr/bin/env python3

import os
import rclpy

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():    # a launcher for text to speech and its tester
    '''Note: Must launch the TTS node alongside another node that uses it for it to function.'''
    stretch_navigation_dir = get_package_share_directory('stretch_nav2')
    stretch_navigation_launch_dir = os.path.join(stretch_navigation_dir, 'launch')
    wvh_guide_dir = get_package_share_directory('wvh_guide_demo')
    wvh_guide_map_dir = os.path.join(wvh_guide_dir, 'svg')

    tts = Node(
            package='parcs_tts',
            namespace='wvh_guide_demo',
            executable='parcs_tts',
            name='parcs_tts',
            output='screen',
            parameters=[{"personality": "you are a helpful robot"}, # the personality if generating responses
                        {"interpreter": "openai"}, # 'festival' or 'openai', case sensitive 
                        {"gen_response": "true"}, # whether you want to generate responses or not: 'true' or 'false', case sensitive
                        {"speaker": "default"}] 
        )
    
    stt = Node(
            package='parcs_stt',
            namespace='wvh_guide_demo',
            executable='parcs_stt',
            name='parcs_stt',
            output='screen',
            parameters=[{"relative_threshold": 0.5}, # relative threshold depending on calibration value, -0.5 works well for quiet areas
                        {"set_threshold": 0.0}, # set the threshold directly without calibration; 0.0 means it will use calibration
                        {"interpreter": 'openai'}, # can be “anthropic” or “openai”
                        {"pause_duration": 2.0}, # the amount of time of a pause in seconds to begin processing audio; the audio chunk length
                        {"microphone": 'default'} # the name of the microphone that you want to be used for detection
                        ] 
        )
    
    directions = Node(
            package='wvh_guide_demo',
            namespace='wvh_guide_demo',
            executable='action_guide',
            name='action_guide',
            output='screen'
        )
    
    navigation = Node(
            package='wvh_guide_demo',
            namespace='wvh',
            executable='navigation',
            name='navigation',
            output='screen'
        )

    chatbot = Node(
            package='wvh_guide_demo',
            namespace='wvh_guide_demo',
            executable='chatbot',
            name='chatbot',
            output='screen',
        )
    
    navigation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(stretch_navigation_launch_dir, 'navigation.launch.py')),
            launch_arguments={
                'map': os.path.join(wvh_guide_map_dir, 'wvh_first_floor.yaml'),
                'autostart': 'true',
                'namespace': 'wvh',
                'use_rviz': 'false',
                'use_namespace': 'true',
                'use_sim_time': 'true'
            }
        )
        
    return LaunchDescription([
        tts,
        stt,
        directions,
        #navigation,
        chatbot,
        # navigation_launch,        
    ])