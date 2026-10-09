import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_drone_gazebo = get_package_share_directory('drone_gazebo')
    pkg_drone_description = get_package_share_directory('drone_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_drone_gazebo, 'worlds', 'border_world.sdf')
    bridge_config_path = os.path.join(pkg_drone_gazebo, 'config', 'drone_1_bridge.yaml')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    xacro_file = os.path.join(pkg_drone_description, 'urdf', 'single_quadrotor.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='drone_1_state_publisher',
        namespace='drone_1',
        output='screen',
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/drone_1/robot_description',
            '-name', 'drone_1',
            '-x', '0', '-y', '0', '-z', '0.2',
        ],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='drone_1_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config_path}],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
    ])