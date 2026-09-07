"""
Yerel EKF: wheel/odometry (mecanum_drive_controller) + IMU gyro Z -> odometry/filtered.
Tek odom->base_footprint TF ureticisi budur (two_d_mode: true, world_frame: odom).
Manyetometre tabanli mutlak yaw KULLANILMAZ (olculmedi).

Onkosul: mecanum_drive_controller aktif (wheel/odometry) ve IMU kaynak
(sim bridge veya BNO055 driver, imu/data_raw) yayinda olmali; degilse EKF
covariance patlar / TF yayinlamaz, bu normaldir (crasha degil, veri bekler).

Kullanim:
  ros2 launch mecanum_navigation ekf.launch.py use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_navigation')
    ekf_params = os.path.join(pkg_share, 'config', 'ekf.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ekf_node,
    ])
