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
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    rap_package_dir = get_package_share_directory('route_admin_panel')
    rosbot_description = get_package_share_directory('rosbot_description')

    rap_server = launch_ros.actions.Node(
        package='route_admin_panel',
        node_executable='node_server.sh',
        output='screen',
        arguments=['-s', 'true', '--min', '10', '--max', '250']
    )

    map_to_img = launch_ros.actions.Node(
        package='route_admin_panel',
        node_executable='map_to_img_node',
        output='screen',
        parameters=[os.path.join(rap_package_dir, 'config', 'map_to_img_node_params.yaml')]
    )

    rosbot_sim = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(rosbot_description, 'launch', 'rosbot_sim.launch.py'))
    )

    rosbot_nav = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(rosbot_description, 'launch', 'rosbot_navigation_sim.launch.py'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        rap_server,
        map_to_img,
        rosbot_sim,
        rosbot_nav,
    ])

if __name__ == '__main__':
    generate_launch_description()
