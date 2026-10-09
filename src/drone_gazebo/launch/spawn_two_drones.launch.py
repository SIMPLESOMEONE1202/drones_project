import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    pkg_drone_gazebo = get_package_share_directory('drone_gazebo')
    pkg_drone_description = get_package_share_directory('drone_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(
        pkg_drone_gazebo,
        'worlds',
        'border_world.sdf'
    )

    drone_1_bridge_config = os.path.join(
        pkg_drone_gazebo,
        'config',
        'drone_1_bridge.yaml'
    )

    drone_2_bridge_config = os.path.join(
        pkg_drone_gazebo,
        'config',
        'drone_2_bridge.yaml'
    )

    xacro_file = os.path.join(
        pkg_drone_description,
        'urdf',
        'single_quadrotor.urdf.xacro'
    )

    # =========================
    # GAZEBO
    # =========================

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}'
        }.items(),
    )

    # =========================
    # DRONE 1 DESCRIPTION
    # =========================

    drone_1_xacro = xacro.process_file(
        xacro_file,
        mappings={
            'drone_name': 'drone_1',
            'x': '0.0',
            'y': '0.0',
            'z': '0.2'
        }
    )

    drone_1_description = {
        'robot_description': drone_1_xacro.toxml()
    }

    # =========================
    # DRONE 2 DESCRIPTION
    # =========================

    drone_2_xacro = xacro.process_file(
        xacro_file,
        mappings={
            'drone_name': 'drone_2',
            'x': '5.0',
            'y': '0.0',
            'z': '0.2'
        }
    )

    drone_2_description = {
        'robot_description': drone_2_xacro.toxml()
    }

    # =========================
    # DRONE 1 STATE PUBLISHER
    # =========================

    drone_1_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='drone_1_state_publisher',
        namespace='drone_1',
        output='screen',
        parameters=[drone_1_description],
    )

    # =========================
    # DRONE 2 STATE PUBLISHER
    # =========================

    drone_2_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='drone_2_state_publisher',
        namespace='drone_2',
        output='screen',
        parameters=[drone_2_description],
    )

    # =========================
    # DRONE 1 SPAWN
    # =========================

    drone_1_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            '/drone_1/robot_description',

            '-name',
            'drone_1',

            '-x',
            '0.0',

            '-y',
            '0.0',

            '-z',
            '0.2',
        ],
        output='screen',
    )

    # =========================
    # DRONE 2 SPAWN
    # =========================

    drone_2_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic',
            '/drone_2/robot_description',

            '-name',
            'drone_2',

            '-x',
            '5.0',

            '-y',
            '0.0',

            '-z',
            '0.2',
        ],
        output='screen',
    )

    # =========================
    # BRIDGES
    # =========================

    drone_1_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='drone_1_bridge',
        output='screen',
        parameters=[
            {
                'config_file': drone_1_bridge_config
            }
        ],
    )

    drone_2_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='drone_2_bridge',
        output='screen',
        parameters=[
            {
                'config_file': drone_2_bridge_config
            }
        ],
    )

    # =========================
    # DELAYED SPAWNING
    # =========================

    delayed_spawning = TimerAction(
        period=3.0,
        actions=[
            drone_1_spawn,
            drone_2_spawn,
        ]
    )

    return LaunchDescription([
        gz_sim,

        drone_1_state_publisher,
        drone_2_state_publisher,

        drone_1_bridge,
        drone_2_bridge,

        delayed_spawning,
    ])