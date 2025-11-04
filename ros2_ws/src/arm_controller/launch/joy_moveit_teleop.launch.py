from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('arm_controller'),
                         'launch', 'demo.launch.py')
        ])
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[{'deadzone': 0.1, 'autorepeat_rate': 20.0}]
    )

    teleop_node = Node(
        package='arm_controller',
        executable='joy_moveit_teleop',
        output='screen'
    )

    return LaunchDescription([
        moveit_demo,
        joy_node,
        teleop_node,
    ])
