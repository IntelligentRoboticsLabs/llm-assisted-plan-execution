import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )
    log_level_arg = DeclareLaunchArgument(
            'log_level',
            default_value=TextSubstitution(text=str('debug')),
            description='Logging level'
    )

    params = os.path.join(
        get_package_share_directory('planning_system'),
        'config',
        'planning_server.yaml'
    )

    planning_node = Node(
        package='planning_system',
        executable='planning_node',
        name='planning_node',
        namespace=namespace,
        output='screen',
        parameters=[params],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        
    )
    return LaunchDescription([namespace_launch_arg, log_level_arg, planning_node])
