"""
Gercek robot orkestrasyonu: description(real) -> controller_manager (ros2_control_node)
-> controllers -> gercek sensorler -> EKF -> localization -> navigation.

Tasima katmani: ESP32 uzerinde micro-ROS calisir; teker hiz komutlari
'wheel/commands' (std_msgs/Float32MultiArray, {FL,FR,RL,RR} rad/s) topic'i ile
gonderilir, teker durumu 'wheel/states' (8 eleman: 4 pozisyon + 4 hiz) ile
geri gelir. Bu launch micro_ros_agent'i BASLATMAZ; agent uzun omurlu ayri bir
surec olarak (systemd veya ayri terminal) calismalidir:

  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32 -b 921600

RPLIDAR driver'i bu launch tarafindan baslatilmaz; IMU ESP32 uzerinden
micro-ROS ile gelir.

transport=mock ile donanimsiz calistirilabilir (bu GERCEK ROBOT TESTI
DEGILDIR). Gercek robotta motor enable etmeden once ESP firmware'in aktif ve
watchdog'lu oldugu ayrica dogrulanmalidir.

Kullanim:
  # donanimsiz arayuz/lifecycle testi, motor hareketsiz
  ros2 launch mecanum_bringup real_nav.launch.py transport:=mock
  # gercek ESP32 (micro_ros_agent ayrica calisiyor olmali)
  ros2 launch mecanum_bringup real_nav.launch.py transport:=micro_ros
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('mecanum_bringup')
    desc_share = get_package_share_directory('mecanum_robot_description')
    control_share = get_package_share_directory('mecanum_control')
    nav_share = get_package_share_directory('mecanum_navigation')

    transport = LaunchConfiguration('transport')
    command_topic = LaunchConfiguration('command_topic')
    state_topic = LaunchConfiguration('state_topic')
    feedback_mode = LaunchConfiguration('feedback_mode')
    environment = LaunchConfiguration('environment')

    xacro_path = os.path.join(desc_share, 'urdf', 'real_robot.xacro')
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_path,
            ' transport:=', transport,
            ' command_topic:=', command_topic,
            ' state_topic:=', state_topic,
            ' feedback_mode:=', feedback_mode,
        ]),
        value_type=str,
    )

    # Tek robot_state_publisher kaynagi: mecanum_robot_description/description.launch.py
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'description.launch.py')),
        launch_arguments={
            'robot_variant': 'real',
            'use_sim_time': 'false',
            'transport': transport,
            'command_topic': command_topic,
            'state_topic': state_topic,
            'feedback_mode': feedback_mode,
        }.items(),
    )

    # ros2_control_node ayrica robot_description'a ihtiyac duyar (Gazebo yok,
    # gz_ros2_control controller_manager'i kendisi olusturmuyor).
    controller_manager_yaml = os.path.join(control_share, 'config', 'mecanum_controllers.yaml')
    # Remap'ler burada verilir (sim'deki ros2_control_sim.xacro <ros><remapping>
    # ile ayni hedef): ros2_control_node BUTUN controller node'larini kendi
    # sureci icinde olusturur, bu yuzden Node()'a verilen remap process-genel
    # uygulanir (headless sim testinde dogrulanan mekanizma).
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_manager_yaml, {'use_sim_time': False}],
        remappings=[
            ('mecanum_drive_controller/odometry', 'wheel/odometry'),
            ('mecanum_drive_controller/reference_unstamped', 'cmd_vel'),
        ],
        output='screen',
    )

    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'controllers.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'ekf.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'localization.launch.py')),
        launch_arguments={'environment': environment, 'use_sim_time': 'false'}.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav_share, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'twist_mux.yaml'),
            {'use_sim_time': False},
        ],
        remappings=[('cmd_vel_out', 'cmd_vel_selected')],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'transport', default_value='mock',
            description="'mock' (guvenli, motor hareketsiz) veya 'micro_ros' (ESP32)"),
        DeclareLaunchArgument('command_topic', default_value='wheel/commands'),
        DeclareLaunchArgument('state_topic', default_value='wheel/states'),
        DeclareLaunchArgument(
            'feedback_mode', default_value='required',
            description="'required' veya 'open_loop' (firmware hazir degilken; odometri gercek degildir)"),
        DeclareLaunchArgument('environment', default_value='katman0'),
        LogInfo(msg=[
            "UYARI: micro_ros_agent ve RPLIDAR driver'i bu launch tarafindan baslatilmiyor. ",
            "transport:=micro_ros kullaniyorsaniz agent ayrica calisiyor olmali; ",
            "scan/imu/data_raw yayinlanmadan AMCL/EKF veri bekler.",
        ]),
        description,
        controller_manager,
        controllers,
        twist_mux_node,
        ekf,
        localization,
        navigation,
    ])
