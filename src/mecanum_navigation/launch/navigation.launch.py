"""
Nav2 navigasyon katmani: controller_server (MPPI), smoother_server (SimpleSmoother
- global plan geometrisi, Velocity Smoother ile KARISTIRILMAZ), planner_server
(SmacPlanner2D), behavior_server, bt_navigator, waypoint_follower + Velocity
Smoother/Collision Monitor (safety_chain.launch.py include edilir — mapping
oturumuyla da paylasilan ortak guvenlik zinciri, kod tekrari yok).

map_server/AMCL/slam_toolbox ICERMEZ (localization.launch.py / mapping.launch.py
ile ayri). Kurulu Humble'in
/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py dosyasi REFERANS
alindi (node listesi, respawn/log_level semantigi) fakat KORLEMESINE include
EDILMEDI: vendor dosyasi velocity_smoother cikisini dogrudan 'cmd_vel'e
remap ediyor (twist_mux ve Collision Monitor'u atliyor). Burada zincir acikca
yeniden kuruldu:

  MPPI (controller_server)  cmd_vel_nav ─┐
                                         ├→ twist_mux (mecanum_bringup) → cmd_vel_selected
  mecanum_teleop             cmd_vel_teleop ─┘
        → velocity_smoother (cmd_vel_selected → cmd_vel_smoothed)      [safety_chain.launch.py]
        → collision_monitor (cmd_vel_smoothed → cmd_vel)               [safety_chain.launch.py]
        → mecanum_drive_controller/reference_unstamped                [mecanum_control remap]

Onkosul: twist_mux'in cmd_vel_selected'i, EKF'nin odometry/filtered'i ve
localization.launch.py'nin map->odom'u yayinda olmalidir (yoksa controller/
AMCL veri bekler, crash olmaz).

Kullanim:
  ros2 launch mecanum_navigation navigation.launch.py use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_share = get_package_share_directory('mecanum_navigation')
    params_file = os.path.join(nav_share, 'params', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time, 'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    navigation_lifecycle_nodes = [
        'controller_server', 'smoother_server', 'planner_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower',
    ]

    controller_server = Node(
        package='nav2_controller', executable='controller_server', output='screen',
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server', name='smoother_server',
        output='screen', parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        # Behavior plugins publish their recovery Twist commands on cmd_vel.
        # Route them through the same Nav2 mux input as MPPI so recoveries
        # cannot bypass Velocity Smoother and Collision Monitor.
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower',
        name='waypoint_follower', output='screen', parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
    )
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': navigation_lifecycle_nodes,
        }],
    )

    safety_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'safety_chain.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'log_level': log_level,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager_navigation,
        safety_chain,
    ])
