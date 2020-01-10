import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rap_package_dir = get_package_share_directory('route_admin_panel')

    rap_server = launch_ros.actions.Node(
        package='route_admin_panel',
        node_executable='node_server.sh',
        output='log'
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        rap_server,
    ])

if __name__ == '__main__':
    generate_launch_description()
