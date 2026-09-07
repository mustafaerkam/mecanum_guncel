"""
RViz2 + Joint State Publisher GUI ile mecanum robotu (Gazebosuz) goruntule.

Kullanim:
  ros2 launch mecanum_robot_description display.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare('mecanum_robot_description').find(
        'mecanum_robot_description'
    )

    xacro_file = PathJoinSubstitution(
        [pkg_share, 'urdf', 'mecanum_robot.xacro']
    )

    rviz_config = os.path.join(pkg_share, 'rviz', 'mecanum_robot.rviz')

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(
        robot_description_content, value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
