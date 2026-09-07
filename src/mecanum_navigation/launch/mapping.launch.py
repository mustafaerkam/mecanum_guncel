"""
Yalniz slam_toolbox mapping oturumu (mode: mapping). map_server ve AMCL
ICERMEZ. localization.launch.py ile AYNI ust agacta calismamalidir — ikisi
de map->odom yayinlar, tek-sahip kuralini bozar (bu launch bunu kendi
basina engelleyemez; mecanum_bringup sim_nav.launch.py / sim_mapping.launch.py
ayrimi bu ikisinin asla birlikte include edilmemesini garanti eder).

Onkosul: simulation.launch.py + mecanum_control/controllers.launch.py calisir
durumda (scan, wheel/odometry, imu/data_raw yayinda) VE mecanum_navigation/
ekf.launch.py aktif olmalidir (slam_toolbox odom_frame'i EKF'nin ciktisini
kullanir).

Kullanim:
  ros2 launch mecanum_navigation mapping.launch.py use_sim_time:=true
Harita kaydetme (harita tamamlaninca):
  ros2 run nav2_map_server map_saver_cli -f /home/<kullanici>/harita_adi
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_navigation')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_node,
    ])
