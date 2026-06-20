from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel'),
        DeclareLaunchArgument('nav_cmd_vel', default_value='/nav_cmd_vel'),
        Node(
            package='g1_cogar_nav_benchmark',
            executable='braitenberg_reflex',
            name='braitenberg_reflex',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
        Node(
            package='g1_cogar_nav_benchmark',
            executable='subsumption_mux',
            name='subsumption_mux',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'nav_cmd_topic': LaunchConfiguration('nav_cmd_vel'),
                'cmd_out_topic': LaunchConfiguration('cmd_vel_out'),
            }],
            output='screen'
        ),
        Node(
            package='g1_cogar_nav_benchmark',
            executable='benchmark_logger',
            name='benchmark_logger',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])
