from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    g1_gazebo_pkg = get_package_share_directory('g1_gazebo')
    world_path = os.path.join(g1_gazebo_pkg, 'worlds', 'empty.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g1_gazebo_pkg, 'launch', 'spawn_g1.launch.py')),
        launch_arguments={'x': '0.0', 'y': '0.0', 'z': '0.0', 'yaw': '0.0', 'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([gazebo, spawn])
