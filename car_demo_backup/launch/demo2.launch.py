#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

def generate_launch_description():
    # 1. Declaramos los argumentos
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    
    # Definimos el argumento 'world' con el mundo por defecto, pero permitiendo cambiarlo
    default_world = os.path.join(get_package_share_directory("car_demo"), 'worlds', 'mcity.world')
    world_arg = DeclareLaunchArgument('world', default_value=default_world, description='Ruta al SDF/World')

    # 2. Configuración de rutas
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_share_directory('car_demo'), "models")
    
    urdf = os.path.join(get_package_share_directory("prius_description"), "urdf", "prius.urdf")
    with open(urdf, "r") as infp:
        robot_desc = infp.read()

    rviz_path = os.path.join(
        get_package_share_directory("car_demo"),
        "rviz", "ros2.rviz"
    )
    print(f"{rviz_path=}")

    return LaunchDescription([
        world_arg,
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('frame', default_value='base_link'),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace="prius",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
            arguments=[urdf]
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", rviz_path,
                "--fixed-frame", LaunchConfiguration(variable_name="frame")
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz"))
        ),

        Node(
            package="car_demo",
            executable="prius_teleop_keyboard.py",
            name="prius_teleop",
            prefix=["xterm -hold -e"],
            output="screen",
        ),

        # Lanzamos el servidor de Gazebo con el mundo que pasemos por parámetro
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'verbose': "true"}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            launch_arguments={'verbose': "true"}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("car_demo"), "launch", "spawn_prius.launch.py")),
            launch_arguments={"pose": "0"}.items()
        )
    ])