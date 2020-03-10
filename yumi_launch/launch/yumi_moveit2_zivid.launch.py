import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode



def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            print('file', file_path, ' opened')
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            print('yaml', file_path, ' opened')
            return yaml.load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        print('yaml', file_path, ' failed to open')
        return None


def generate_launch_description():

    pkgShareDir  = get_package_share_directory('yumi_launch')

    configDir_L    = os.path.join(pkgShareDir, 'config', 'yumi_params_L_sim.yaml')
    configDir_R    = os.path.join(pkgShareDir, 'config', 'yumi_params_R_sim.yaml')
    
    urdf = os.path.join(get_package_share_directory('yumi_description'), 'urdf', 'yumi.urdf')
    assert os.path.exists(urdf)

    robot_description_config = load_file('yumi_description', 'urdf/yumi.urdf')
    robot_description = {'robot_description' : robot_description_config}

    rviz_config_dir = os.path.join(get_package_share_directory('yumi_description'), 'config', 'yumi.rviz')
    assert os.path.exists(rviz_config_dir)


    # Globals
    yumi_robot_manager = Node(package= 'yumi_robot_manager',
                              node_executable='yumi_robot_manager_node')
    
    global_joint_state = Node(package='yumi_sim',
                              node_executable='global_joint_state_node',
                              output='screen')


    # Left Arm
    abb_egm_hardware_sim_left = Node(package= 'abb_egm_hardware',
                                     node_executable='abb_egm_hardware_node',
                                     node_namespace='/l',
                                     arguments=['/l'],
                                     output='screen',
                                     parameters=[os.path.join(get_package_share_directory("yumi_launch"), "config", "yumi_left_controllers.yaml"),
                                                 os.path.join(get_package_share_directory("yumi_launch"), "config", "start_positions_left.yaml")])
    
    param_server_left =  Node(package='parameter_server', 
                              node_executable='param_server_node',
                              node_namespace='/l', 
                              arguments=[configDir_L])                              
    
    sg_control_left = Node(package='sg_control', 
                              node_executable='sg_control_node',
                              node_namespace='/l') 


    # Right Arm
    abb_egm_hardware_sim_right = Node(package= 'abb_egm_hardware',
                                      node_executable='abb_egm_hardware_node',
                                      node_namespace='/r',
                                      arguments=['/r'],
                                      output='screen',
                                      parameters=[os.path.join(get_package_share_directory("yumi_launch"), "config", "yumi_right_controllers.yaml"),
                                                  os.path.join(get_package_share_directory("yumi_launch"), "config", "start_positions_right.yaml")])
    
    param_server_right = Node(package='parameter_server', 
                              node_executable='param_server_node',
                              node_namespace='/r', 
                              arguments=[configDir_R])                              
    
    sg_control_right = Node(package='sg_control', 
                              node_executable='sg_control_sim_node',
                              node_namespace='/r') 


    # RViz
    rviz_node = Node(package='rviz2',
                     node_executable='rviz2',
                     node_name='rviz2',
                     arguments=['-d', rviz_config_dir],
                     parameters=[robot_description],
                     output='screen')

    # Publish base link TF
    static_tf = Node(package='tf2_ros',
                     node_executable='static_transform_publisher',
                     node_name='static_transform_publisher',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'yumi_base_link', 'yumi_body'])
    

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        node_name='pose_estimation_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='pose_estimation',
                node_plugin='pose_estimation::PoseEstimation',
                node_name='pose_estimation',
            ),
        ],
        output='screen',
    )
     # zivid manual composition
    zivid_camera = Node(package='zivid_camera',
             node_executable='manual_composition',
             parameters=[{"zivid.camera.num_capture_frames" : 3}],
             output='screen')

    state_publisher = Node(package='robot_state_publisher',
                            node_executable='robot_state_publisher',
                            node_name='robot_state_publisher',
                            output='screen',
                            arguments=[urdf])

    return LaunchDescription([ zivid_camera, rviz_node, static_tf,
                               yumi_robot_manager, global_joint_state,
                               abb_egm_hardware_sim_left, param_server_left, sg_control_left,
                               abb_egm_hardware_sim_right, param_server_right, sg_control_right,
                               container, state_publisher])
   
      
      
