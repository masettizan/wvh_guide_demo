#! /usr/bin/env python3


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():    # a launcher for text to speech and its tester
    '''Note: Must launch the TTS node alongside another node that uses it for it to function.'''

    return LaunchDescription([
        Node(
            package='parcs_tts',
            namespace='wvh_guide_demo',
            executable='parcs_tts',
            name='parcs_tts',
            output='screen',
            parameters=[{"personality": "you are a helpful robot"}, # the personality if generating responses
                        {"interpreter": "openai"}, # 'festival' or 'openai', case sensitive 
                        {"gen_response": "false"}, # whether you want to generate responses or not: 'true' or 'false', case sensitive
                        {"speaker": "default"}] 
        ),
        Node(
            package='wvh_guide_demo',
            namespace='wvh_guide_demo',
            executable='action_guide',
            name='action_guide',
            output='screen'
        ),
        Node(
            package='wvh_guide_demo',
            namespace='wvh_guide_demo',
            executable='chatbot',
            name='chatbot',
            output='screen',
        ),
        
    ])