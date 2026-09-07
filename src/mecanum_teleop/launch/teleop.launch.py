"""
Klavye teleop dugumu — YALNIZ cmd_vel_teleop yayinlar.

DIKKAT: BU LAUNCH DOSYASI TELEOP ICIN CALISMAZ.
'ros2 launch' alt surecin stdin'ini kendi surec yonetimine baglar;
emulate_tty=True yalniz CIKTI icin pty acar, girdi icin degil. Node'un
termios/tty cagrilari bu durumda ENOTTY ("Inappropriate ioctl for device")
ile patlar. teleop_node bunu basta tespit edip acik bir hatayla cikar.

DOGRU KULLANIM — kendi interaktif terminalinizde:
  ros2 run mecanum_teleop teleop_node

Dosya, launch'in neden kullanilamadigini kayit altina almak icin duruyor;
baska launch'lar tarafindan include EDILMEZ.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package='mecanum_teleop',
        executable='teleop_node',
        name='mecanum_teleop',
        output='screen',
        emulate_tty=True,
    )
    return LaunchDescription([teleop_node])
