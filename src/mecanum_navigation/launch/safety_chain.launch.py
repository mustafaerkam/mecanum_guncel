"""
Velocity Smoother + Collision Monitor — son yazilimsal guvenlik zinciri.
navigation.launch.py VE sim_mapping.launch.py/real_mapping.launch.py TARAFINDAN
ORTAK kullanilir: mapping oturumunda da (Nav2 stack'i kapaliyken) teleop
mux/smoother/Collision Monitor'u bypass EDEMEZ (kilitli kural).

Zincir: cmd_vel_selected (twist_mux) -> velocity_smoother -> cmd_vel_smoothed
-> collision_monitor -> cmd_vel (mecanum_drive_controller/reference_unstamped'e
remap mecanum_control/controllers.launch.py'de yapilir).

Onkosul: twist_mux'in cmd_vel_selected ciktisi yayinda olmali (mecanum_bringup).

'stop_polygon_enabled' argumani (varsayilan true) PolygonStop'u ac/kapa eder.
Zincirin kendisi HER ZAMAN kurulur; kapatilan yalniz stop poligonudur,
PolygonSlow ve velocity_smoother etkilenmez.

Kullanim:
  ros2 launch mecanum_navigation safety_chain.launch.py use_sim_time:=true
  ros2 launch mecanum_navigation safety_chain.launch.py stop_polygon_enabled:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_share = get_package_share_directory('mecanum_navigation')
    params_file = os.path.join(nav_share, 'params', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    stop_polygon_enabled = LaunchConfiguration('stop_polygon_enabled')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time, 'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother',
        name='velocity_smoother', output='screen', parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[('cmd_vel', 'cmd_vel_selected'), ('cmd_vel_smoothed', 'cmd_vel_smoothed')],
    )
    lifecycle_manager_smoother = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_velocity_smoother', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['velocity_smoother'],
        }],
    )

    # PolygonStop'u launch seviyesinde ac/kapa. ROS 2'de ic ice YAML
    # parametreleri nokta ile adlandirilir, bu yuzden 'PolygonStop.enabled'
    # nav2_params.yaml'daki degeri dogrudan ezer (parameters listesinde
    # SONRA gelen kazanir). YAML'a dokunulmaz.
    #
    # NEDEN: PolygonStop'un 'stop' aksiyonu twist'in TAMAMINI sifirlar
    # (vx, vy, wz) ve YON BILGISI YOKTUR. Robot stop bolgesine girdiginde
    # duvardan UZAKLASAN komut da sifirlanir, yani KILITLENIR ve operator
    # kurtaramaz. Olculdu (2026-08-24, katman1): kilitliyken duvardan
    # uzaklasma denemesi 0.000 m yer degistirme, komutlarin %99'u sifir;
    # PolygonStop kapatilinca ayni komut 0.290 m.
    #
    # Mapping oturumunda Nav2 yoktur, direksiyonda operator vardir ve
    # carpismayi Gazebo fizigi zaten engeller; bu yuzden sim_mapping
    # PolygonStop'u kapatir. PolygonSlow (slowdown) ACIK kalir — o twist'i
    # sifirlamaz, yalnizca olceklendirir, dolayisiyla kilitlemez.
    #
    # VARSAYILAN 'true': navigation.launch.py bu argumani GECMEZ, boylece
    # Nav2 (sim ve gercek) yolunda stop korumasi aynen korunur.
    collision_monitor = Node(
        package='nav2_collision_monitor', executable='collision_monitor',
        name='collision_monitor', output='screen',
        parameters=[
            configured_params,
            {'PolygonStop.enabled': ParameterValue(stop_polygon_enabled,
                                                   value_type=bool)},
        ],
        arguments=['--ros-args', '--log-level', log_level],
    )
    lifecycle_manager_collision_monitor = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor', output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['collision_monitor'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'stop_polygon_enabled', default_value='true',
            description="Collision Monitor PolygonStop ('stop' aksiyonu) acik mi. "
                        "Nav2 yolunda DAIMA true kalmalidir; yalniz mapping "
                        "oturumu false gecer (bkz. collision_monitor Node yorumu)."),
        velocity_smoother,
        lifecycle_manager_smoother,
        collision_monitor,
        lifecycle_manager_collision_monitor,
    ])
