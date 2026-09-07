# S_Mecanum_Nav

Dört tekerlekli mecanum robot için ROS 2 Humble + Gazebo Fortress + Nav2.
Simülasyon ve gerçek robot aynı üst kontrol, topic ve TF mimarisini kullanır.

## Kurulum

Doğrulanan geliştirme ortamı: Ubuntu 22.04, ROS 2 Humble, Gazebo Fortress,
FastRTPS. Önce ROS 2 Humble ve Gazebo Fortress depolarını kurun.

```bash
git clone git@github.com:mustafaerkam/mecanum_guncel.git
cd mecanum_guncel
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-twist-mux \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-mecanum-drive-controller ros-humble-gz-ros2-control \
  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
# rosdep ilk kez kullanılacaksa: sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Her terminalde workspace içinde:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=1
export GZ_VERSION=fortress
```

`ROS_LOCALHOST_ONLY=1` aynı bilgisayardaki süreçler içindir; uzaktaki RViz
veya operatör bilgisayarı için ağ yapılandırması ayrıca yapılmalıdır.

## Simülasyon

Kayıtlı demo haritasıyla navigasyon:

```bash
ros2 launch mecanum_bringup sim_nav.launch.py
```

Haritalama için navigasyon oturumunu kapatıp:

```bash
ros2 launch mecanum_bringup sim_mapping.launch.py stop_polygon_enabled:=true
# Ayrı terminal:
ros2 launch mecanum_teleop teleop.launch.py
# Haritayı kaydet:
ros2 run nav2_map_server map_saver_cli -f /tmp/mecanum_map \
  --ros-args -p use_sim_time:=true
```

Haritalama oturumunu kapattıktan sonra yeni haritayla navigasyon:

```bash
ros2 launch mecanum_bringup sim_nav.launch.py \
  map:=/tmp/mecanum_map.yaml auto_initial_pose:=false
```

RViz'de önce `2D Pose Estimate`, ardından `Nav2 Goal` verin. Görsel arayüz
istenmiyorsa `headless:=true rviz:=false` ekleyin. SLAM ve AMCL oturumlarını
aynı anda çalıştırmayın. Oturumları `Ctrl+C` ile kapatın.

## Mimari

```text
Nav2 / teleop → twist_mux → velocity_smoother → collision_monitor
             → mecanum_drive_controller → ros2_control → dört teker

wheel/odometry + imu/data_raw → EKF → odometry/filtered
```

- MPPI: `Omni`; AMCL: `OmniMotionModel`; global planner: SmacPlanner2D.
- `map → odom`: SLAM sırasında slam_toolbox, navigasyonda AMCL.
- `odom → base_footprint`: yalnız EKF.
- Gövde/sensör/teker TF'leri: robot_state_publisher.
- Final `cmd_vel`: yalnız Collision Monitor.
- Gerçek donanım: Pi 5 → USB/micro-ROS → tek ESP32 → dört motor;
  BNO055 ESP32'ye I²C, RPLIDAR Pi'ye USB ile bağlanır.
- Motor PID, encoder okuma ve watchdog ESP32 firmware'inin sorumluluğudur.

## Gerçek donanım arayüzü

**Gerçek araç üzerinde doğrulanmadı; ESP32 firmware'i bu depoda bulunmuyor.**
Pi kurulumu için işletim sistemi/ROS dağıtımı uyumu ayrıca doğrulanmalıdır;
yukarıdaki kurulum doğrulanan Humble geliştirme ortamına aittir.

| Yön | Topic | Mesaj |
|---|---|---|
| Pi → ESP | `wheel/commands` | `Float32MultiArray`: FL, FR, RL, RR hedef hızları, rad/s; 50 Hz |
| ESP → Pi | `wheel/states` | `Float32MultiArray`: dört kümülatif pozisyon (rad), ardından dört hız (rad/s) |
| ESP → Pi | `imu/data_raw` | `sensor_msgs/Imu`, `frame_id: imu_link` |

Varsayılan QoS best-effort, depth 1. Agent, firmware seri hızı, zaman
senkronizasyonu ve topic sözleşmesi birbiriyle uyumlu olmalıdır.

Donanımsız arayüz denemesi:

```bash
ros2 launch mecanum_bringup real_nav.launch.py transport:=mock
```

Gerçek taşıma seçimi `transport:=micro_ros` şeklindedir. `micro_ros_agent`
ve LiDAR sürücüsü ayrıca kurulup başlatılmalıdır; launch bunları oluşturmaz.
`/dev/esp32` ve `/dev/lidar` adları udev ile tanımlanmalıdır. Gerçek başlangıç
pozu manueldir. `feedback_mode:=open_loop` gerçek odometri sağlamaz.

## Doğrulama ve bilinen sınırlar

```bash
ros2 run mecanum_testing contract_audit
```

2026-09-08 simülasyon testinde SLAM haritası kaydedilip AMCL/Nav2'ye yüklendi;
4/4 hedef başarılı, 0 recovery. LiDAR yaklaşık 5.52 Hz, IMU 100 Hz,
teker odometrisi 50 Hz ve EKF 30 Hz ölçüldü. Bu, elektronik veya Pi performans
doğrulaması değildir. `motion_probe` yalnız izole Gazebo testi içindir;
gerçek araçta tek teker testi olarak kullanılmamalıdır.
