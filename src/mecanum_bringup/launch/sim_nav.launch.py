"""
Ust orkestrasyon: simulation -> controllers -> EKF -> localization -> navigation
-> twist_mux; opsiyonel RViz. AMCL modu (kayitli harita). Canli SLAM icin
sim_mapping.launch.py kullanin — ikisi AYNI ANDA calistirilmamalidir (ikisi
de map->odom yayinlar).

controller_manager, gz_ros2_control plugin'i (sim_robot.xacro) tarafindan
Gazebo sureci icinde olusturulur; controllers.launch.py'deki spawner'lar
kendi --controller-manager-timeout'lariyla bu servisin hazir olmasini
BEKLER (sabit TimerAction/sleep DEGIL). EKF/localization/navigation veri
gelene kadar bekler (crash olmaz), bu normaldir.

Onkosul: yok (en ust katman). Uyumsuz: sim_mapping.launch.py ile ayni anda
calistirilamaz (her ikisi de map->odom TF sahibidir).

Kullanim:
  ros2 launch mecanum_bringup sim_nav.launch.py
  ros2 launch mecanum_bringup sim_nav.launch.py environment:=katman0 rviz:=false
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
    map_yaml = LaunchConfiguration('map')
    rviz = LaunchConfiguration('rviz')
    headless = LaunchConfiguration('headless')
    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')

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

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'localization.launch.py')),
        launch_arguments={
            'environment': environment,
            # Empty keeps the profile map.  A saved SLAM map can be supplied
            # here without changing the world/profile contract.
            'map': map_yaml,
            'use_sim_time': 'true',
            # SIM-ONLY: AMCL baslangic pozunu otomatik ver. Gazebo'da spawn
            # pozu bilindigi icin RViz "2D Pose Estimate" zorunlulugu
            # simulasyonda gereksiz bir el islemi. Poz, haritanin nasil
            # uretildigine gore turetilir (bkz. localization.launch.py
            # _cozulen_baslangic_pozu): profil haritasinda spawn_pose,
            # kullanicinin SLAM haritasinda (0,0,0).
            # Gercek robot yolu (real_nav.launch.py) bu argumani GECMEZ,
            # orada manuel poz kilitli karari surer.
            'auto_initial_pose': auto_initial_pose,
            'initial_pose_x': initial_pose_x,
            'initial_pose_y': initial_pose_y,
            'initial_pose_yaw': initial_pose_yaw,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
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
        DeclareLaunchArgument(
            'map', default_value='',
            description='Saved SLAM map .yaml absolute path; empty uses the environment profile map'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'auto_initial_pose', default_value='true',
            description='AMCL baslangic pozunu otomatik ver (sim). false '
                        'yaparsaniz RViz "2D Pose Estimate" ile elle verirsiniz.'),
        DeclareLaunchArgument(
            'initial_pose_x', default_value='',
            description='Acik baslangic pozu X (MAP frame); bos ise turetilir'),
        DeclareLaunchArgument(
            'initial_pose_y', default_value='',
            description='Acik baslangic pozu Y (MAP frame); bos ise turetilir'),
        DeclareLaunchArgument(
            'initial_pose_yaw', default_value='',
            description='Acik baslangic yaw (MAP frame); bos ise turetilir'),
        simulation,
        controllers,
        ekf,
        localization,
        navigation,
        twist_mux_node,
        rviz_node,
    ])
