import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkgShareDir  = get_package_share_directory('yumi_launch')

    configDir_L    = os.path.join(pkgShareDir, 'config', 'yumi_params_L.yaml')
    configDir_R    = os.path.join(pkgShareDir, 'config', 'yumi_params_R.yaml')
    

    return LaunchDescription([

      ################# Global node group ###################
      # Global RWS robot manager
      Node(package= 'abb_robot_manager',
                              node_executable='yumi_robot_manager_node',
                              #arguments=['egm'],    # egm / rapid
                              output='screen'),
      

      ################ Left arm node group ##################
      # Left arm
      Node(package= 'abb_egm_hardware',
                              node_executable='abb_egm_hardware_node',
                              node_namespace='/l',
                              arguments=[configDir_L, '/l'],
                              output='screen'),

      # Left arm paramter server
      Node(package='parameter_server', 
                              node_executable='param_server_exec',
                              node_namespace='/l', 
                              arguments=[configDir_L],     
                              output='screen'),
      
      # Left Smart Gripper action server
      Node(package='sg_control', 
                              node_executable='sg_control_node',
                              node_namespace='/l',   
                              output='screen'),


      ################ Right arm node group ##################
      # Right arm
      Node(package= 'abb_egm_hardware',
                              node_executable='abb_egm_hardware_node',
                              node_namespace='/r',
                              arguments=[configDir_R, '/r'],
                              output='screen'),

      # Right arm parameter server
      Node(package='parameter_server', 
                              node_executable='param_server_exec',
                              node_namespace='/r', 
                              arguments=[configDir_R],     
                              output='screen'),

      # Right Smart Gripper action server
      Node(package='sg_control', 
                              node_executable='sg_control_node',
                              node_namespace='/r',   
                              output='screen'),

      #######################################################                              
    ])
  
        