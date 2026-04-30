import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('car_demo')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]),

        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[nav2_params]),

        Node(
            package='car_demo',
            executable='parking_detector.py',
            name='parking_detector',
            output='screen',
            parameters=[{'use_sim_time': True}]),

        Node(
            package='car_demo',
            executable='parking_controller.py',
            name='parking_controller',
            output='screen',
            parameters=[{'use_sim_time': True}]),

        # Node(
        #     package='nav2_behaviors',
        #     executable='behavior_server',
        #     name='behavior_server',
        #     output='screen',
        #     parameters=[nav2_params]),

        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[
        #         nav2_params,
        #         {'default_bt_xml_filename': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_only_if_goal_is_updated.xml'}
        #     ]),


        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        # {'node_names': ['planner_server', 'controller_server', 'behavior_server', 'bt_navigator']}])
                        {'node_names': ['planner_server']}])
    ])