"""
Tek robot_state_publisher tanimi — sim VE gercek robotun tek robot_description
kaynagi. robot_variant secimine gore sim_robot.xacro (gz_ros2_control) veya
real_robot.xacro (mecanum_control/MecanumSystemInterface) xacro'sunu acar.

Onkosul: yok (temel katman). use_sim_time varsayilani false'tur; ust sim
launch'lar (mecanum_simulation/simulation.launch.py) acikca true gecirir.

Kullanim:
  ros2 launch mecanum_robot_description description.launch.py robot_variant:=sim use_sim_time:=true
  ros2 launch mecanum_robot_description description.launch.py robot_variant:=real transport:=mock
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mecanum_robot_description')
    robot_variant = LaunchConfiguration('robot_variant').perform(context)

    if robot_variant not in ('sim', 'real'):
        raise RuntimeError(f"robot_variant 'sim' veya 'real' olmali, alinan: '{robot_variant}'")

    xacro_name = 'sim_robot.xacro' if robot_variant == 'sim' else 'real_robot.xacro'
    xacro_path = os.path.join(pkg_share, 'urdf', xacro_name)

    cmd = ['xacro ', xacro_path]
    if robot_variant == 'real':
        cmd += [
            ' transport:=', LaunchConfiguration('transport'),
            ' command_topic:=', LaunchConfiguration('command_topic'),
            ' state_topic:=', LaunchConfiguration('state_topic'),
            ' feedback_mode:=', LaunchConfiguration('feedback_mode'),
        ]
    robot_description = ParameterValue(Command(cmd), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
    )
    return [robot_state_publisher]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_variant', default_value='sim',
            description="'sim' (gz_ros2_control) veya 'real' (SocketCAN SystemInterface)"),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'transport', default_value='mock',
            description="robot_variant=real icin: 'mock' (donanimsiz) veya 'micro_ros' (ESP32)"),
        DeclareLaunchArgument('command_topic', default_value='wheel/commands'),
        DeclareLaunchArgument('state_topic', default_value='wheel/states'),
        DeclareLaunchArgument(
            'feedback_mode', default_value='required',
            description="'required' veya 'open_loop' (firmware hazir degilken; odometri gercek degildir)"),
        OpaqueFunction(function=_launch_setup),
    ])
