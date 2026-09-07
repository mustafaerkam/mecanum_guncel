"""
Ust orkestrasyon: simulation -> controllers -> EKF -> mapping (slam_toolbox)
-> guvenli teleop hatti (twist_mux -> safety_chain: Velocity Smoother ->
Collision Monitor). Nav2 (controller/planner/bt_navigator) YOKTUR — mapping
sirasinda navigasyon calismaz (kilitli kural). Teleop yine de mux/smoother/
Collision Monitor'u bypass EDEMEZ; bu yuzden Nav2'nin geri kalani olmasa
bile safety_chain.launch.py ayrica include edilir.

Onkosul: yok (en ust katman). Uyumsuz: sim_nav.launch.py ile ayni anda
calistirilamaz (her ikisi de map->odom TF sahibidir: slam_toolbox vs AMCL).

mecanum_teleop BURADA BASLATILMAZ: klavye/tty girisi ayri bir terminal
gerektirir (launch icinde stdin paylasimi guvenilir degildir). Ayri terminalde:
  ros2 launch mecanum_teleop teleop.launch.py

Kullanim:
  ros2 launch mecanum_bringup sim_mapping.launch.py
  ros2 launch mecanum_bringup sim_mapping.launch.py environment:=katman0
Harita kaydetme:
  ros2 run nav2_map_server map_saver_cli -f /home/<kullanici>/harita_adi
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    environment = LaunchConfiguration('environment')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    stop_polygon = LaunchConfiguration('stop_polygon_enabled')

    sim_share = get_package_share_directory('mecanum_simulation')
    control_share = get_package_share_directory('mecanum_control')
    nav_share = get_package_share_directory('mecanum_navigation')
    bringup_share = get_package_share_directory('mecanum_bringup')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, 'launch', 'simulation.launch.py')),
        launch_arguments={'environment': environment, 'headless': headless}.items(),
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'controllers.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'ekf.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'mapping.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # stop_polygon_enabled=false: MAPPING oturumuna OZGU.
    # Collision Monitor'un 'stop' aksiyonu twist'in tamamini sifirlar ve yon
    # bilgisi yoktur; robot stop bolgesine girdiginde duvardan uzaklasan komut
    # da sifirlandigi icin operator KURTARAMAZ (olculdu 2026-08-24, katman1:
    # kilitliyken kacis 0.000 m, PolygonStop kapaliyken ayni komut 0.290 m).
    # Mapping'de Nav2 yok, direksiyonda operator var, carpismayi Gazebo fizigi
    # engelliyor. PolygonSlow ACIK kalir (slowdown kilitlemez).
    # Nav2 oturumu (sim_nav/real_nav -> navigation.launch.py) bu argumani
    # GECMEZ, orada stop korumasi aynen surer.
    safety_chain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'safety_chain.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'stop_polygon_enabled': stop_polygon,
        }.items(),
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'twist_mux.yaml'),
            {'use_sim_time': True},
        ],
        remappings=[('cmd_vel_out', 'cmd_vel_selected')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'mecanum_nav.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('environment', default_value='katman0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'stop_polygon_enabled', default_value='false',
            description='Mapping oturumunda Collision Monitor PolygonStop. '
                        'Varsayilan false: stop aksiyonu operatoru duvar '
                        'dibinde kilitliyor. true yaparak geri acabilirsiniz.'),
        simulation,
        controllers,
        ekf,
        mapping,
        safety_chain,
        twist_mux_node,
        rviz_node,
    ])
