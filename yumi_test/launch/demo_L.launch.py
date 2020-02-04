import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    joint = launch.substitutions.LaunchConfiguration('joint', default=0)

    return LaunchDescription([
    

      Node(package= 'testing_tools',  
           node_executable='demo_L',
           node_namespace='/l',
           arguments=[joint],
           output='screen')

    ])
    
