from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('g1_description_ros2')
    default_model = os.path.join(pkg_share, 'urdf', 'g1_nav.urdf')

    model_arg = DeclareLaunchArgument(
        'model',
        default_value=default_model,
        description='URDF model to display in RViz.',
    )

    model = LaunchConfiguration('model')
    robot_description = Command(['cat ', model])

    return LaunchDescription([
        model_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
