#!/usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a YAML file from a package"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"[WARN] Could not load YAML: {absolute_file_path} ({e})")
        return None


def launch_setup(context, *args, **kwargs):
    # --- Package directories ---
    moveit_config_pkg = get_package_share_directory('arm_controller')
    urdf_pkg = get_package_share_directory('sekirei_urdf')

    # --- Load URDF ---
    urdf_file = os.path.join(urdf_pkg, 'urdf', 'sekirei_moveit.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # --- Load SRDF ---
    srdf_file = os.path.join(urdf_pkg, 'srdf', 'sekirei.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # --- Load MoveIt config YAMLs ---
    kinematics_yaml = load_yaml('arm_controller', 'config/kinematics.yaml')
    ompl_planning_yaml = load_yaml('arm_controller', 'config/ompl_planning.yaml')
    moveit_cpp_yaml = load_yaml('arm_controller', 'config/moveit_cpp.yaml')
    moveit_controllers = load_yaml('arm_controller', 'config/moveit_controllers.yaml')

    # --- Build move_group parameters ---
    move_group_params = {
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'use_sim_time': False,
    }

    if kinematics_yaml:
        move_group_params['robot_description_kinematics'] = kinematics_yaml
    if ompl_planning_yaml:
        move_group_params.update(ompl_planning_yaml)
    if moveit_cpp_yaml:
        move_group_params.update(moveit_cpp_yaml)
    if moveit_controllers:
        move_group_params.update(moveit_controllers)

    # --- Core Nodes ---

    # Fake controller (publishes /joint_states)
    fake_controller_node = Node(
        package='arm_controller',
        executable='fake_controller.py',
        output='screen',
        name='fake_controller',
    )

    # MoveIt move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[move_group_params],
    )

    # RViz2
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{
            'robot_description': robot_description,
            'robot_description_semantic': robot_description_semantic,
            'robot_description_kinematics': kinematics_yaml if kinematics_yaml else {},
        }],
    )

    # Static TF world→base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}],
    )

    # Joystick input (ros-joy)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
    )

    # C++ joy teleop node
    joy_teleop_node = Node(
        package='arm_controller',
        executable='joy_moveit_teleop',
        name='joy_moveit_teleop',
        output='screen',
    )

    return [
        static_tf_node,
        robot_state_publisher,
        fake_controller_node,
        move_group_node,
        rviz_node,
        joy_node,
        joy_teleop_node,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
