from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString
import os

from g1_cogar_nav_benchmark.planner_profiles import DEFAULT_PROFILES
from g1_cogar_nav_benchmark.scenario_library import load_scenarios


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('g1_cogar_nav_benchmark')
    g1_gazebo_share = get_package_share_directory('g1_gazebo')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    scenario_file = LaunchConfiguration('scenario_file').perform(context)
    scenario_id = LaunchConfiguration('scenario_id').perform(context)
    planner_id = LaunchConfiguration('planner_id').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    use_reflex = LaunchConfiguration('use_reflex')
    teb_plugin_class = LaunchConfiguration('teb_plugin_class').perform(context).strip()

    scenarios = load_scenarios(scenario_file)
    if scenario_id not in scenarios:
        raise RuntimeError(f'Unknown scenario_id: {scenario_id}')
    if planner_id not in DEFAULT_PROFILES:
        raise RuntimeError(f'Unknown planner_id: {planner_id}')

    scenario = scenarios[scenario_id]
    profile = DEFAULT_PROFILES[planner_id]

    world_path = scenario.world
    params_file_path = os.path.join(pkg_share, profile.params_file)
    bt_xml_path = os.path.join(pkg_share, 'config', f'nav_to_pose_{planner_id}_humble.xml')
    if planner_id == 'teb' and not teb_plugin_class:
        raise RuntimeError(
            'planner_id:=teb requires teb_plugin_class:=<installed Nav2 TEB controller plugin class>. '
            'DWB and MPPI are available without this external plugin.'
        )
    params_file = ReplaceString(
        source_file=params_file_path,
        replacements={
            '<NAV_TO_POSE_BT_XML>': bt_xml_path,
            '<TEB_PLUGIN_CLASS>': teb_plugin_class,
        },
    )
    spawn_launch = os.path.join(g1_gazebo_share, 'launch', 'spawn_g1.launch.py')
    gazebo_launch = os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
    nav_launch = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    nav_cmd_target = LaunchConfiguration('nav_cmd_vel').perform(context)

    actions = [
        LogInfo(msg=f'Launching scenario={scenario_id}, planner={planner_id}, run={LaunchConfiguration("run_id").perform(context)}'),
        LogInfo(msg=f'World={world_path}'),
        LogInfo(msg=f'Map={scenario.map_yaml}'),
        LogInfo(msg=f'Planner notes: {profile.notes}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world_path,
                'verbose': 'true',
                'gui': LaunchConfiguration('use_gazebo_gui'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={
                'x': str(scenario.start_pose[0]),
                'y': str(scenario.start_pose[1]),
                'z': '0.1',
                'yaw': str(scenario.start_pose[2]),
                'entity': 'g1',
                'use_sim_time': use_sim_time.perform(context),
            }.items(),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_baseline_localization',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': scenario.map_yaml}],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart, 'node_names': ['map_server']}],
        ),
        TimerAction(
            period=35.0,
            actions=[
                GroupAction([
                    SetRemap('cmd_vel', nav_cmd_target),
                    SetRemap('/cmd_vel', nav_cmd_target),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(nav_launch),
                        launch_arguments={
                            'use_sim_time': use_sim_time.perform(context),
                            'params_file': params_file,
                            'autostart': autostart.perform(context),
                            'use_composition': 'False',
                        }.items(),
                    ),
                ])
            ],
        ),
        Node(
            package='g1_cogar_nav_benchmark',
            executable='benchmark_logger',
            name='benchmark_logger',
            parameters=[{'use_sim_time': use_sim_time, 'output_file': LaunchConfiguration('logger_output_file')}],
            output='screen',
        ),
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='g1_cogar_nav_benchmark',
                    executable='g1_robot_simulation.py',
                    name='g1_robot_simulation',
                    parameters=[{
                        'use_sim_time': use_sim_time,
                        'model_name': 'g1',
                        'service_name': '/gazebo/set_model_configuration',
                        'trajectory_topic': LaunchConfiguration('gazebo_joint_trajectory_topic'),
                        'use_gazebo_trajectory': LaunchConfiguration('use_gazebo_joint_trajectory'),
                        'use_set_model_configuration': LaunchConfiguration('use_set_model_configuration'),
                    }],
                    condition=IfCondition(LaunchConfiguration('enable_g1_robot_simulation')),
                    output='screen',
                )
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen',
        ),
        Node(
            package='g1_cogar_nav_benchmark',
            executable='braitenberg_reflex',
            name='braitenberg_reflex',
            condition=IfCondition(use_reflex),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
        Node(
            package='g1_cogar_nav_benchmark',
            executable='subsumption_mux',
            name='subsumption_mux',
            parameters=[{
                'use_sim_time': use_sim_time,
                'nav_cmd_topic': LaunchConfiguration('nav_cmd_vel'),
                'cmd_out_topic': LaunchConfiguration('cmd_vel_out'),
            }],
            output='screen',
        ),
    ]

    if scenario.dynamic_obstacles:
        actions.append(
            Node(
                package='g1_cogar_nav_benchmark',
                executable=scenario.dynamic_obstacle_cmd,
                name='dynamic_obstacle_commander',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('g1_cogar_nav_benchmark')
    default_scenario_file = os.path.join(pkg_share, 'config', 'benchmark_scenarios.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'benchmark.rviz')

    ld = LaunchDescription()
    for name, default, description in [
        ('scenario_file', default_scenario_file, 'Scenario configuration YAML file.'),
        ('scenario_id', 'doorway', 'Scenario id to launch.'),
        ('planner_id', 'dwb', 'Planner profile id (dwb, mppi, teb).'),
        ('run_id', 'manual', 'Run label used in logs and saved result folders.'),
        ('teb_plugin_class', '', 'Installed Nav2-compatible TEB controller plugin class used when planner_id:=teb.'),
        ('use_sim_time', 'true', 'Use Gazebo simulation clock.'),
        ('use_rviz', 'true', 'Launch RViz2.'),
        ('autostart', 'true', 'Autostart Nav2 lifecycle nodes.'),
        ('use_reflex', 'false', 'Enable Braitenberg reflex avoidance. The final command mux stays enabled as a stop watchdog.'),
        ('enable_g1_robot_simulation', 'true', 'Animate leg and arm joints in Gazebo using /cmd_vel-synchronised fake walking.'),
        ('use_gazebo_joint_trajectory', 'true', 'Drive Gazebo visual joints through gazebo_ros_joint_pose_trajectory.'),
        ('use_set_model_configuration', 'false', 'Legacy Gazebo joint animation backend using /gazebo/set_model_configuration.'),
        ('gazebo_joint_trajectory_topic', '/g1/set_joint_trajectory', 'JointTrajectory topic consumed by the Gazebo joint pose trajectory plugin.'),
        ('cmd_vel_out', '/cmd_vel', 'Final command topic sent to the robot.'),
        ('nav_cmd_vel', '/nav_cmd_vel', 'Intermediate Nav2 command topic routed through the final command mux.'),
        ('logger_output_file', '/tmp/k3_latest_log.csv', 'Telemetry CSV path written by benchmark_logger.'),
        ('rviz_config', default_rviz, 'RViz2 configuration file.'),
        ('use_gazebo_gui', 'false', 'Launch Gazebo client GUI (false for headless benchmark runs).'),
    ]:
        ld.add_action(DeclareLaunchArgument(name, default_value=default, description=description))

    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
