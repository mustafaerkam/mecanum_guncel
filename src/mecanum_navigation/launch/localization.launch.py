"""
map_server + AMCL + kendi lifecycle_manager_localization'i. slam_toolbox
ICERMEZ (mapping.launch.py ile ayni ust agacta olmamali). map->odom'un
navigasyon oturumundaki tek sahibi AMCL'dir.

'map' argumani BOS OLAMAZ: verilmezse 'environment' argumanindan
(mecanum_navigation/profiles/environments.yaml) cozulur, boylece world-map
uyumsuzlugu launch aninda acikca patlar (dosya yoksa RuntimeError).

set_initial_pose: false (nav2_params.yaml) — bu launch /initialpose
YAYINLAMAZ; baslangic pozu RViz "2D Pose Estimate" ile verilir. Yalniz
environment profilindeki nominal spawn_pose'u log'a bilgi amacli yazar.

Composition (ComposableNode) ILK ASAMADA desteklenmiyor; Pi 5 performans
performans olcumunden sonra degerlendirilecek.

Onkosul: simulation.launch.py (veya gercekte real sensor bringup) + EKF
calisir, /scan yayinda olmali.

Kullanim:
  ros2 launch mecanum_navigation localization.launch.py environment:=katman0
  ros2 launch mecanum_navigation localization.launch.py map:=/mutlak/yol/harita.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
import yaml


def _profil(context):
    """environments.yaml'dan aktif environment profilini dondurur."""
    environment_name = LaunchConfiguration('environment').perform(context)
    nav_share = get_package_share_directory('mecanum_navigation')
    profiles_path = os.path.join(nav_share, 'profiles', 'environments.yaml')
    with open(profiles_path, 'r') as f:
        data = yaml.safe_load(f)
    environments = data.get('environments', {})
    if environment_name not in environments:
        raise RuntimeError(
            f"Bilinmeyen environment '{environment_name}'. "
            f"Gecerli environment'lar: {list(environments.keys())}"
        )
    return environment_name, environments[environment_name]


def _cozulen_baslangic_pozu(context):
    """AMCL baslangic pozunu MAP frame'inde cozer (auto_initial_pose icin).

    KRITIK AYRIM — map frame'i nerede oldugu haritanin NASIL uretildigine bagli:

      * Profil haritasi (map argumani BOS): repodaki hazir harita dunya
        frame'ine hizalidir (katman0.yaml kapsami dunya duvar ic yuzleri
        +-3.00 ile ortusuyor). Bu durumda
        map frame == dunya frame, dolayisiyla baslangic pozu = profil
        spawn_pose'udur.

      * Kullanicinin SLAM ile urettigi harita (map argumani VERILMIS):
        slam_toolbox map frame'ini SLAM'in BASLADIGI poza yerlestirir.
        Mapping oturumu spawn'da basladigi icin robot, map frame'inde
        (0, 0, 0) noktasindadir.

    Bu ikisi karistirilirsa AMCL spawn kadar (katman0'da 1.5 m) hatali
    baslar. Acik override icin initial_pose_x/y/yaw kullanin.
    """
    ex = LaunchConfiguration('initial_pose_x').perform(context).strip()
    ey = LaunchConfiguration('initial_pose_y').perform(context).strip()
    eyaw = LaunchConfiguration('initial_pose_yaw').perform(context).strip()
    if ex or ey or eyaw:
        return (float(ex or 0.0), float(ey or 0.0), float(eyaw or 0.0),
                'acik override (initial_pose_x/y/yaw)')

    if LaunchConfiguration('map').perform(context):
        return (0.0, 0.0, 0.0,
                'kullanici SLAM haritasi -> map frame SLAM baslangicinda, '
                'robot (0,0,0)')

    _, profil = _profil(context)
    sp = profil['spawn_pose']
    return (float(sp['x']), float(sp['y']), float(sp.get('yaw', 0.0)),
            'profil haritasi -> map frame == dunya frame, poz = spawn_pose')


def _resolve_map_path(context):
    map_arg = LaunchConfiguration('map').perform(context)
    if map_arg:
        return map_arg

    nav_share = get_package_share_directory('mecanum_navigation')
    _, profil = _profil(context)
    map_file = profil['map_file']
    map_path = os.path.join(nav_share, 'maps', map_file)
    if not os.path.isfile(map_path):
        raise RuntimeError(f"Harita dosyasi bulunamadi: {map_path}")
    return map_path


def _launch_setup(context, *args, **kwargs):
    nav_share = get_package_share_directory('mecanum_navigation')
    params_file = os.path.join(nav_share, 'params', 'nav2_params.yaml')

    map_yaml_path = _resolve_map_path(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_path},
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = ['map_server', 'amcl']

    # auto_initial_pose: SIM-ONLY kolaylik. VARSAYILAN 'false' — gercek robot
    # yolu (real_nav.launch.py) bu argumani GECMEZ, orada kilitli karar
    # (set_initial_pose: false, RViz "2D Pose Estimate") aynen surer.
    # Gercek robotta bilinen bir spawn pozu yoktur, bu yuzden otomatik poz
    # ORADA ANLAMSIZDIR. Yalniz sim_nav.launch.py 'true' gecer.
    auto_poz = LaunchConfiguration('auto_initial_pose').perform(context).lower() == 'true'
    amcl_ek_params = {}
    poz_notu = "kapali — RViz '2D Pose Estimate' ile manuel verin"
    if auto_poz:
        px, py, pyaw, gerekce = _cozulen_baslangic_pozu(context)
        amcl_ek_params = {
            'set_initial_pose': True,
            'initial_pose.x': px,
            'initial_pose.y': py,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': pyaw,
        }
        poz_notu = f"otomatik: map frame'inde ({px:+.3f}, {py:+.3f}, {pyaw:+.3f}) — {gerekce}"

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
    )

    # amcl_ek_params configured_params'tan SONRA gelir; ROS 2'de sonra gelen
    # kazanir, boylece nav2_params.yaml'daki set_initial_pose: false degeri
    # YAML'a dokunmadan ezilir.
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params, amcl_ek_params] if amcl_ek_params
        else [configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes,
        }],
    )

    return [
        LogInfo(msg=f'[localization] harita: {map_yaml_path}'),
        LogInfo(msg=f'[localization] baslangic pozu: {poz_notu}'),
        map_server,
        amcl,
        lifecycle_manager,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'environment', default_value='katman0',
            description='map bos ise environments.yaml profilinden harita cozulur'),
        DeclareLaunchArgument(
            'map', default_value='',
            description='Mutlak harita .yaml yolu; bos ise environment profilinden alinir'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument(
            'auto_initial_pose', default_value='false',
            description='AMCL baslangic pozunu otomatik ver (SIM-ONLY). '
                        'VARSAYILAN false: gercek robot yolu kilitli karari '
                        '(RViz 2D Pose Estimate) korur. sim_nav.launch.py true gecer.'),
        DeclareLaunchArgument(
            'initial_pose_x', default_value='',
            description='Acik baslangic pozu X (MAP frame). Bos ise turetilir.'),
        DeclareLaunchArgument(
            'initial_pose_y', default_value='',
            description='Acik baslangic pozu Y (MAP frame). Bos ise turetilir.'),
        DeclareLaunchArgument(
            'initial_pose_yaw', default_value='',
            description='Acik baslangic yaw (MAP frame). Bos ise turetilir.'),
        OpaqueFunction(function=_launch_setup),
    ])
