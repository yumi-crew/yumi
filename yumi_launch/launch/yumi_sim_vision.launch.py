import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    pkgShareDir = get_package_share_directory('yumi_launch')

    configDir_L = os.path.join(pkgShareDir, 'config', 'yumi_params_L_sim.yaml')
    configDir_R = os.path.join(pkgShareDir, 'config', 'yumi_params_R_sim.yaml')

    urdf = os.path.join(get_package_share_directory(
        'yumi_description'), 'urdf', 'yumi.urdf')
    assert os.path.exists(urdf)

    rviz_config_dir = os.path.join(get_package_share_directory(
        'yumi_description'), 'config', 'yumi_zivid.rviz')
    print(rviz_config_dir)    
    assert os.path.exists(rviz_config_dir)

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        node_name='pose_estimation_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
                # package='zivid_camera',
                # node_plugin='zivid_camera::ZividCamera',
                # node_name='zivid_camera',
                # parameters=[{"zivid.camera.num_capture_frames" : 3}]),
            ComposableNode(
                package='pose_estimation',
                node_plugin='pose_estimation::PoseEstimation',
                node_name='pose_estimation',
            ),
        ],
        output='screen',
    )

    return LaunchDescription([

        ################# Global node group ###################
        # Global RWS robot manager
        Node(package='yumi_robot_manager',
             node_executable='yumi_robot_manager_sim_node',
             output='screen'),


        ################ Left arm node group ##################
        # Left arm
        Node(package='abb_egm_hardware',
             node_executable='abb_egm_hardware_sim_node',
             node_namespace='/l',
             arguments=['/l'],
             output='screen'),

        # Left arm paramter server
        Node(package='parameter_server',
             node_executable='param_server_node',
             node_namespace='/l',
             arguments=[configDir_L],
             output='screen'),

        # Left Smart Gripper action server
        Node(package='sg_control',
             node_executable='sg_control_sim_node',
             node_namespace='/l',
             output='screen'),


        ################ Right arm node group ##################
        # Right arm
        Node(package='abb_egm_hardware',
             node_executable='abb_egm_hardware_sim_node',
             node_namespace='/r',
             arguments=['/r'],
             output='screen'),

        # Right arm parameter server
        Node(package='parameter_server',
             node_executable='param_server_node',
             node_namespace='/r',
             arguments=[configDir_R],
             output='screen'),

        # Right Smart Gripper action server
        Node(package='sg_control',
             node_executable='sg_control_sim_node',
             node_namespace='/r',
             output='screen'),

        # zivid manual composition
        Node(package='zivid_camera',
             node_executable='manual_composition',
             parameters=[{"zivid.camera.num_capture_frames" : 3}],
             output='screen'),
        ################ Rviz launch ##################
        Node(package='rviz2',
             node_executable='rviz2',
             node_name='rviz2',
             arguments=['-d', rviz_config_dir],
             output='screen'
             ),
        Node(package='robot_state_publisher',
             node_executable='robot_state_publisher',
             node_name='robot_state_publisher',
             output='screen',
             arguments=[urdf]),
        Node(package='ros2_control_utils',
             node_executable='global_joint_state_node_sim',
             node_name='global_joint_state_node',
             output='screen',
             arguments=[]),
        container
    ])
