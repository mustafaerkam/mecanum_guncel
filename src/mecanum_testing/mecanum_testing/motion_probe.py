#!/usr/bin/env python3
"""Izole hareket kabul testi (test-only, uretim launch'ina ait DEGILDIR).

Gazebo + ros2_control + mecanum_drive_controller'in EN KUCUK ortamında
gercek gövde hareketini ground truth ile olcer. SLAM/EKF/twist_mux/
Velocity Smoother/Collision Monitor bu testte YOKTUR; amac temas fizigini
nav yigininden ayirmaktir.

OLCUM KURALI: abonelik callback'leri AYRI bir executor thread'inde doner.
Tek bir spin_once dongusunden okuma yapmak, birden fazla abonelik arasinda
callback aclığı yaratip BAYAT ground-truth mesaji okunmasina yol acar
(bu hata bir kez yapildi ve yanlis "capraz kayma" sonucu uretti). Her
ornekte mesaj timestamp'i de kaydedilir, bayatlik acikca raporlanir.

Kullanim:
  ros2 run mecanum_testing motion_probe --out /tmp/.../motion.json
"""
import argparse
import json
import math
import statistics
import threading
import time
from datetime import datetime, timezone

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

WHEEL_RADIUS = 0.1625          # xacro wheel_radius
LXLY = 0.94                    # controller sum_of_robot_center_projection_on_X_Y_axis
JOINT_ORDER = ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class MotionProbe(Node):

    def __init__(self, cmd_topic):
        super().__init__('motion_probe')
        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.lock = threading.Lock()
        self.gt = None
        self.wheel = None
        self.js = None
        self.create_subscription(Odometry, '/ground_truth/odom', self._gt, qos)
        self.create_subscription(Odometry, '/wheel/odometry', self._wh, qos)
        self.create_subscription(JointState, '/joint_states', self._js, qos)

    def _gt(self, m):
        with self.lock:
            self.gt = m

    def _wh(self, m):
        with self.lock:
            self.wheel = m

    def _js(self, m):
        with self.lock:
            self.js = m

    def sample(self):
        """Anlik ornek; her kaynagin kendi timestamp'i ile birlikte."""
        with self.lock:
            gt, wh, js = self.gt, self.wheel, self.js
        if gt is None or wh is None or js is None:
            return None
        vel = dict(zip(js.name, js.velocity))
        return {
            'wall': time.time(),
            'gt': {
                'stamp': gt.header.stamp.sec + gt.header.stamp.nanosec * 1e-9,
                'x': gt.pose.pose.position.x, 'y': gt.pose.pose.position.y,
                'z': gt.pose.pose.position.z, 'yaw': yaw_of(gt.pose.pose.orientation),
            },
            'odom': {
                'stamp': wh.header.stamp.sec + wh.header.stamp.nanosec * 1e-9,
                'x': wh.pose.pose.position.x, 'y': wh.pose.pose.position.y,
                'yaw': yaw_of(wh.pose.pose.orientation),
            },
            'joints': {n: vel.get(n, float('nan')) for n in JOINT_ORDER},
        }

    def hold(self, twist, seconds, rate_hz=50.0, record=False):
        """Sabit frekansta yayinla; istege bagli zaman serisi kaydet."""
        series = []
        dt = 1.0 / rate_hz
        t0 = time.time()
        nxt = t0
        while time.time() - t0 < seconds:
            self.pub.publish(twist)
            if record:
                s = self.sample()
                if s is not None:
                    series.append(s)
            nxt += dt
            time.sleep(max(0.0, nxt - time.time()))
        return series

    def wait_ready(self, timeout=30.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.sample() is not None:
                return True
            time.sleep(0.1)
        return False

    def settle(self, seconds):
        """Komutsuz oturma; Z kararliligini olc."""
        s = self.hold(Twist(), seconds, record=True)
        z = [p['gt']['z'] for p in s]
        return {
            'sure_s': seconds, 'ornek': len(z),
            'z_ilk': z[0], 'z_son': z[-1],
            'z_min': min(z), 'z_max': max(z),
            'z_bant_m': max(z) - min(z),
            'z_std_m': statistics.pstdev(z) if len(z) > 1 else 0.0,
            'x_son': s[-1]['gt']['x'], 'y_son': s[-1]['gt']['y'],
            'yaw_son': s[-1]['gt']['yaw'],
        }


def run_case(node, name, vx, vy, wz, seconds, settle_s):
    node.hold(Twist(), settle_s)                       # once tam dur
    a = node.sample()
    tw = Twist()
    tw.linear.x, tw.linear.y, tw.angular.z = vx, vy, wz
    series = node.hold(tw, seconds, record=True)
    stop_series = node.hold(Twist(), 2.0, record=True)  # komut kesilince durma
    b = series[-1]

    # KRITIK: Twist komutu base_footprint (govde) frame'indedir. Dunya
    # frame'indeki ham dx/dy ile karsilastirmak, robot onceki testten kalan
    # bir yaw ile basladiginda yanlis sonuc verir (bir kez yasandi: saf yana
    # kayma, yaw=1.5 rad'da dunyada -x hareketi olarak gorundu). Delta'yi
    # baslangic yaw'ina gore govde frame'ine dondururuz.
    def to_body(dx, dy, yaw0):
        c, s_ = math.cos(yaw0), math.sin(yaw0)
        return (c * dx + s_ * dy, -s_ * dx + c * dy)

    gdx_w = b['gt']['x'] - a['gt']['x']
    gdy_w = b['gt']['y'] - a['gt']['y']
    gdx, gdy = to_body(gdx_w, gdy_w, a['gt']['yaw'])
    gdth = ang_diff(b['gt']['yaw'], a['gt']['yaw'])
    odx_w = b['odom']['x'] - a['odom']['x']
    ody_w = b['odom']['y'] - a['odom']['y']
    odx, ody = to_body(odx_w, ody_w, a['odom']['yaw'])
    odth = ang_diff(b['odom']['yaw'], a['odom']['yaw'])
    el = b['wall'] - series[0]['wall']

    # gövde frame'inde beklenen yer degistirme (yaw ~ sabit varsayimi degil:
    # saf donuste lineer beklenti zaten sifir)
    exp_x, exp_y, exp_th = vx * el, vy * el, wz * el

    # komut kesildikten sonra kalan hareket
    c = stop_series[-1]
    coast = math.hypot(c['gt']['x'] - b['gt']['x'], c['gt']['y'] - b['gt']['y'])

    # teker hizlari: komutun ortasindaki plato
    mid = series[len(series) // 2]['joints']
    exp_wheel = None
    if abs(vy) < 1e-9 and abs(wz) < 1e-9:
        exp_wheel = vx / WHEEL_RADIUS
    elif abs(vx) < 1e-9 and abs(vy) < 1e-9:
        exp_wheel = wz * LXLY / WHEEL_RADIUS

    zs = [p['gt']['z'] for p in series]
    # ground truth mesaj bayatligi (wall clock ile stamp farki degil,
    # ardisik orneklerde stamp'in gercekten ilerledigi kontrolu)
    stamps = [p['gt']['stamp'] for p in series]
    tekrar = sum(1 for i in range(1, len(stamps)) if stamps[i] == stamps[i - 1])

    res = {
        'ad': name,
        'komut': {'vx': vx, 'vy': vy, 'wz': wz, 'sure_s': seconds},
        'olculen_sure_s': round(el, 3),
        'ornek_sayisi': len(series),
        'bayat_ornek_orani': round(tekrar / max(1, len(series) - 1), 3),
        'baslangic': {'x': a['gt']['x'], 'y': a['gt']['y'], 'yaw': a['gt']['yaw'], 'z': a['gt']['z']},
        'bitis': {'x': b['gt']['x'], 'y': b['gt']['y'], 'yaw': b['gt']['yaw'], 'z': b['gt']['z']},
        'ground_truth_delta_govde': {'dx': round(gdx, 4), 'dy': round(gdy, 4), 'dyaw': round(gdth, 4)},
        'ground_truth_delta_dunya': {'dx': round(gdx_w, 4), 'dy': round(gdy_w, 4)},
        'wheel_odom_delta_govde': {'dx': round(odx, 4), 'dy': round(ody, 4), 'dyaw': round(odth, 4)},
        'beklenen_delta': {'dx': round(exp_x, 4), 'dy': round(exp_y, 4), 'dyaw': round(exp_th, 4)},
        'z_bant_m': round(max(zs) - min(zs), 5),
        'komut_sonrasi_kayma_m': round(coast, 4),
        'teker_hizlari_orta': {k: round(v, 4) for k, v in mid.items()},
        'beklenen_teker_hizi': round(exp_wheel, 4) if exp_wheel is not None else None,
    }

    # izleme hatasi ve odometri-gercek farki
    if abs(wz) > 1e-9 and abs(vx) < 1e-9 and abs(vy) < 1e-9:
        res['hedef_buyukluk'] = round(abs(exp_th), 4)
        res['gercek_buyukluk'] = round(abs(gdth), 4)
        res['odom_gt_fark'] = round(abs(odth - gdth), 4)
        base = abs(gdth)
    else:
        res['hedef_buyukluk'] = round(math.hypot(exp_x, exp_y), 4)
        res['gercek_buyukluk'] = round(math.hypot(gdx, gdy), 4)
        res['odom_gt_fark'] = round(math.hypot(odx - gdx, ody - gdy), 4)
        base = math.hypot(gdx, gdy)
    res['izleme_hatasi_m'] = round(abs(res['gercek_buyukluk'] - res['hedef_buyukluk']), 4)
    res['odom_gt_fark_yuzde'] = round(100.0 * res['odom_gt_fark'] / base, 1) if base > 1e-6 else None
    return res, series


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True, help='JSON sonuc dosyasi')
    ap.add_argument('--cmd-topic', default='/cmd_vel')
    ap.add_argument('--settle', type=float, default=5.0)
    ap.add_argument('--trace', default=None, help='ham zaman serisi JSON (opsiyonel)')
    args = ap.parse_args()

    rclpy.init()
    node = MotionProbe(args.cmd_topic)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()

    if not node.wait_ready():
        print('HATA: /ground_truth/odom, /wheel/odometry veya /joint_states gelmedi')
        rclpy.shutdown()
        return 1

    print(f'== komutsuz oturma ({args.settle}s) ==')
    settle = node.settle(args.settle)
    print(f"  z: {settle['z_ilk']:.4f} -> {settle['z_son']:.4f}  "
          f"bant={settle['z_bant_m']:.5f} m  std={settle['z_std_m']:.5f} m")
    print(f"  poz: x={settle['x_son']:+.3f} y={settle['y_son']:+.3f} yaw={settle['yaw_son']:+.3f}")

    cases = [
        ('A_ileri',   0.15,  0.0,  0.0,  5.0),
        ('B_geri',   -0.15,  0.0,  0.0,  3.0),
        ('C_donus',   0.0,   0.0,  0.25, 6.0),
        ('D_yana',    0.0,   0.15, 0.0,  5.0),
    ]
    results = []
    traces = {}
    for name, vx, vy, wz, sec in cases:
        r, series = run_case(node, name, vx, vy, wz, sec, settle_s=3.0)
        results.append(r)
        traces[name] = series
        print(f"\n-- {name}  komut(vx={vx} vy={vy} wz={wz}) {sec}s --")
        print(f"   gt(govde)   dx={r['ground_truth_delta_govde']['dx']:+.4f} "
              f"dy={r['ground_truth_delta_govde']['dy']:+.4f} "
              f"dyaw={r['ground_truth_delta_govde']['dyaw']:+.4f}")
        print(f"   odom(govde) dx={r['wheel_odom_delta_govde']['dx']:+.4f} "
              f"dy={r['wheel_odom_delta_govde']['dy']:+.4f} "
              f"dyaw={r['wheel_odom_delta_govde']['dyaw']:+.4f}")
        print(f"   beklenen buyukluk={r['hedef_buyukluk']:.4f}  gercek={r['gercek_buyukluk']:.4f}  "
              f"izleme hatasi={r['izleme_hatasi_m']:.4f}")
        print(f"   odom-gt fark={r['odom_gt_fark']:.4f} ({r['odom_gt_fark_yuzde']}%)  "
              f"z bant={r['z_bant_m']:.5f} m  komut sonrasi kayma={r['komut_sonrasi_kayma_m']:.4f} m")
        print(f"   teker: {r['teker_hizlari_orta']}  beklenen={r['beklenen_teker_hizi']}")
        print(f"   bayat ornek orani={r['bayat_ornek_orani']}")

    out = {
        'zaman_utc': datetime.now(timezone.utc).isoformat(),
        'not': 'Izole test: SLAM/EKF/twist_mux/smoother/collision_monitor YOK.',
        'oturma': settle,
        'testler': results,
    }
    with open(args.out, 'w') as f:
        json.dump(out, f, indent=2)
    print(f'\nJSON yazildi: {args.out}')
    if args.trace:
        with open(args.trace, 'w') as f:
            json.dump(traces, f)
        print(f'Ham zaman serisi: {args.trace}')

    ex.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
