import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Cambia 'niagara_model' por el nombre real de tu paquete
    pkg_share = get_package_share_directory('niagara_model')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['planner_server']}]),

        Node(
            package='niagara_model',
            executable='parking_detector.py',
            name='parking_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]),

        Node(
            package='niagara_model',
            executable='parking_controller.py',
            name='parking_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]),
    ])