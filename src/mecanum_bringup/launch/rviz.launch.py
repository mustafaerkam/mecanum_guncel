"""
Bagimsiz RViz baslatici — mecanum_nav.rviz config'i (LaserScan, Map, AMCL
particles, TF, plan, local/global costmap, published footprint, Collision
Monitor stop/slowdown poligonlari dahil). sim_nav.launch.py/sim_mapping.launch.py
zaten kendi RViz'ini rviz:=true ile acabilir; bu dosya RViz'i sonradan ayri
baslatmak/yeniden baslatmak icindir.

Kullanim:
  ros2 launch mecanum_bringup rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('mecanum_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'mecanum_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rviz_node,
    ])
