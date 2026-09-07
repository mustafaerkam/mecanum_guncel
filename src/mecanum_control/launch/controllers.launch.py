"""
Var olan controller_manager'a joint_state_broadcaster ve mecanum_drive_controller'i
spawn eder. controller_manager'i KENDI BASINA baslatmaz:
  - Simde: gz_ros2_control plugin'i (sim_robot.xacro) controller_manager'i Gazebo
    icinde kendisi olusturur.
  - Gercekte: real_hardware.launch.py (mecanum_bringup) ayri bir
    ros2_control_node/controller_manager baslatir.
Bu launch, controller_manager servisi hazir olana kadar bekleyip sirayla
joint_state_broadcaster -> mecanum_drive_controller spawn eder; joint_state_broadcaster
active olmadan mecanum_drive_controller spawn edilmez (sim ve gercekte ayni semantik).

Onkosul: controller_manager node'u zaten calisir ve /controller_manager/list_controllers
servisi mevcut olmalidir (spawner bunu kendi timeout'u ile bekler).

Kullanim:
  ros2 launch mecanum_control controllers.launch.py
  ros2 launch mecanum_control controllers.launch.py use_sim_time:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
        output='screen',
    )

    # NOT: mecanum_drive_controller/odometry -> wheel/odometry ve
    # mecanum_drive_controller/reference_unstamped -> cmd_vel remap'leri
    # BURADA VERILMEZ. spawner yalniz controller_manager'a servis cagirir,
    # controller'in gercek node'unu OLUSTURMAZ; spawner'a verilen remap'lerin
    # hicbir etkisi olmadigi headless testte "Publisher count: 0" ile
    # bulundu. Gercek remap: sim'de mecanum_control/urdf/ros2_control_sim.xacro
    # (<ros><remapping>), gercekte mecanum_bringup/launch/real_nav.launch.py
    # (ros2_control_node Node'unun remappings'i).
    mecanum_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager-timeout', '30'],
        output='screen',
    )

    # joint_state_broadcaster spawner surecinin BASARIYLA cikmasi (active olmasi)
    # BEKLENDIKTEN SONRA mecanum_drive_controller spawn edilir. OnProcessStart degil
    # OnProcessExit kullanilir cunku spawner sadece controller active olunca cikar.
    spawn_mecanum_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_drive_controller_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        joint_state_broadcaster_spawner,
        spawn_mecanum_after_jsb,
    ])
