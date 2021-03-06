import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


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

    moveit_cpp_yaml_file_name = get_package_share_directory('yumi_description') + "/moveit2_config/moveit_cpp.yaml"

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file('yumi_description', 'urdf/yumi.urdf')
    robot_description = {'robot_description' : robot_description_config}
    robot_description_path = {'robot_description_path' : os.path.join(get_package_share_directory('yumi_description'), 'urdf/yumi.urdf')}

    robot_description_semantic_config = load_file('yumi_description', 'moveit2_config/yumi.srdf')
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    kinematics_yaml = load_yaml('yumi_description', 'moveit2_config/kinematics.yaml')
    robot_description_kinematics = { 'robot_description_kinematics' : kinematics_yaml }

    controllers_yaml = load_yaml('yumi_description', 'moveit2_config/controllers.yaml')
    moveit_controllers = { 'moveit_simple_controller_manager' : controllers_yaml }

    joint_limits_yaml = load_yaml('yumi_description', 'moveit2_config/joint_limits.yaml')
    joint_limits = {'robot_description_planning' : joint_limits_yaml}

    ompl_planning_pipeline_config = { 'ompl' : {
        'planning_plugin' : 'ompl_interface/OMPLPlanner',
        'request_adapters' : """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""" ,
        'start_state_max_bounds_error' : 0.1 } }
    ompl_planning_yaml = load_yaml('yumi_description', 'moveit2_config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    #demo_dual
    demo_dual = Node(package='yumi_test',
                        node_executable='demo_dual',
                        output='screen',
                        parameters=[moveit_cpp_yaml_file_name,
                                    robot_description,
                                    robot_description_path,
                                    robot_description_semantic,
                                    kinematics_yaml,
                                    joint_limits,
                                    ompl_planning_pipeline_config,
                                    moveit_controllers])

    return LaunchDescription([ demo_dual ])
