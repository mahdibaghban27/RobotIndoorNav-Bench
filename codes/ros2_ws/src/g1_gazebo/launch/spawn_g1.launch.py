from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    g1_desc_pkg = get_package_share_directory('g1_description_ros2')
    g1_gazebo_pkg = get_package_share_directory('g1_gazebo')
    model_path = os.path.join(g1_desc_pkg, 'urdf', 'g1_nav.urdf')
    scan_cfg = os.path.join(g1_gazebo_pkg, 'config', 'pointcloud_to_scan.yaml')
    share_root = os.path.dirname(g1_desc_pkg)

    with open(model_path, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('entity', default_value='g1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[EnvironmentVariable('GAZEBO_MODEL_PATH', default_value=''), ':', share_root],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        TimerAction(
            period=20.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    output='screen',
                    arguments=[
                        '-entity', LaunchConfiguration('entity'),
                        '-topic', 'robot_description',
                        '-x', LaunchConfiguration('x'),
                        '-y', LaunchConfiguration('y'),
                        '-z', LaunchConfiguration('z'),
                        '-Y', LaunchConfiguration('yaw'),
                        '-timeout', '90',
                    ],
                )
            ],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='mid360_to_scan',
            output='screen',
            parameters=[scan_cfg, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[('cloud_in', '/livox/points'), ('scan', '/scan')],
        ),
    ])
