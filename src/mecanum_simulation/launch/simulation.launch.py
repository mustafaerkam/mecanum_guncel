"""
Gazebo Fortress (Ignition Gazebo 6) baslatir, sim robotu (mecanum_robot_description
sim_robot.xacro + mecanum_control gz_ros2_control) spawn eder ve YALNIZ Gazebo-native
veriyi (clock, scan, imu, ground-truth odom) ROS'a kopruler.

controller_manager'i bu launch ACMAZ: gz_ros2_control plugin'i (sim_robot.xacro
icinde) Gazebo sureci icinde kendi controller_manager'ini olusturur. Teker
controller'larini spawn etmek icin ayrica mecanum_control/controllers.launch.py
calistirilmalidir (mecanum_bringup/sim_nav.launch.py bunu sirayla yapar).

Onkosul: yok (ilk katman). Uyumsuz: navigation.launch.py'den ONCE calismali.

Kullanim:
  ros2 launch mecanum_simulation simulation.launch.py
  ros2 launch mecanum_simulation simulation.launch.py environment:=katman0 headless:=true
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_environment_profile(environment_name: str) -> dict:
    nav_share = get_package_share_directory('mecanum_navigation')
    profiles_path = os.path.join(nav_share, 'profiles', 'environments.yaml')
    with open(profiles_path, 'r') as f:
        data = yaml.safe_load(f)
    environments = data.get('environments', {})
    if environment_name not in environments:
        raise RuntimeError(
            f"Bilinmeyen environment '{environment_name}'. Gecerli degerler: "
            f"{list(environments.keys())} (bkz. mecanum_navigation/profiles/environments.yaml)"
        )
    return environments[environment_name]


def _launch_setup(context, *args, **kwargs):
    environment_name = LaunchConfiguration('environment').perform(context)
    profile = _load_environment_profile(environment_name)

    sim_share = get_package_share_directory('mecanum_simulation')
    desc_share = get_package_share_directory('mecanum_robot_description')

    world_path = os.path.join(sim_share, 'worlds', profile['world_file'])
    if not os.path.isfile(world_path):
        raise RuntimeError(f"World dosyasi bulunamadi: {world_path}")

    headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')
    gz_args = ['-r', world_path] + (['-s'] if headless else [])

    set_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=':'.join(filter(None, [
            os.path.dirname(desc_share),
            os.path.join(desc_share, 'meshes'),
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
        ])),
    )

    # ros-humble-gz-ros2-control bu degiskeni KENDISI EXPORT ETMIYOR (headless
    # testte "Failed to load system plugin [gz_ros2_control-system]: couldn't
    # find shared library" hatasiyla bulundu). AMENT_PREFIX_PATH'teki her
    # prefix'in lib/ altini Fortress'in (Ignition Gazebo 6) plugin arama
    # yoluna ekliyoruz.
    ament_prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(':')
    plugin_lib_dirs = ':'.join(filter(None, [os.path.join(p, 'lib') for p in ament_prefixes if p]))
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=':'.join(filter(None, [
            plugin_lib_dirs, os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ])),
    )
    set_ign_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value=':'.join(filter(None, [
            plugin_lib_dirs, os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
        ])),
    )

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo'] + gz_args,
        output='screen',
        emulate_tty=True,
    )

    # Tek robot_state_publisher kaynagi: mecanum_robot_description/description.launch.py
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'description.launch.py')
        ),
        launch_arguments={
            'robot_variant': 'sim',
            'use_sim_time': 'true',
        }.items(),
    )

    spawn_pose = profile['spawn_pose']
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'mecanum_robot',
            '-x', str(spawn_pose['x']),
            '-y', str(spawn_pose['y']),
            '-z', str(spawn_pose['z']),
            '-Y', str(spawn_pose['yaw']),
        ],
        parameters=[{'use_sim_time': True}],
    )

    bridge_config = os.path.join(sim_share, 'config', 'gz_bridge.yaml')
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [
        set_resource_path,
        set_gz_plugin_path,
        set_ign_plugin_path,
        gz_sim,
        description,
        parameter_bridge,
        spawn_robot,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'environment', default_value='katman0',
            description='mecanum_navigation/profiles/environments.yaml icindeki profil adi'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description="true ise 'ign gazebo -s' (sunucu-only, GUI yok)"),
        OpaqueFunction(function=_launch_setup),
    ])
