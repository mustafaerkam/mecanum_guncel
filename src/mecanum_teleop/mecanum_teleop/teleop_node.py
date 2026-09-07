#!/usr/bin/env python3
"""
teleop_node.py — Mecanum robot icin klavye kontrol (v2, mecanum_teleop paketi)

cmd_vel_teleop yayinlar (DOGRUDAN cmd_vel DEGIL): twist_mux -> Velocity
Smoother -> Collision Monitor zincirini bypass etmez.

YENILIKLER (v1'e gore):
  - Ayri lineer/acisal hiz ayari (donus cok hizliysa sadece onu dusur)
  - Hassas mod (SHIFT yerine 'F' tusu): tum hizlar yariya iner
  - 90 derece donus yardimcisi: 'K' basili tutmadan tam donus
  - Anlik durum gostergesi

Tus duzeni:

    Q   W   E        capraz-sol-ileri | ileri | capraz-sag-ileri
    A   S   D        yana-sol         | DUR   | yana-sag
    Z   X   C        capraz-sol-geri  | geri  | capraz-sag-geri

    J / L            sola don / saga don (yavas, kontrollu)
    U / O            sola/saga 90 derece don (otomatik)

    1 / 2            lineer hiz  -/+
    3 / 4            acisal hiz  -/+
    F                hassas mod ac/kapa (hizlar yariya iner)
    S veya BOSLUK    DUR
    CTRL-C           cikis
"""

import os
import sys
import termios
import tty
import select
import time
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# (x_yonu, y_yonu, donme_yonu)
TUS_HARITASI = {
    'w': ( 1.0,  0.0,  0.0),
    'x': (-1.0,  0.0,  0.0),
    'a': ( 0.0,  1.0,  0.0),
    'd': ( 0.0, -1.0,  0.0),
    'q': ( 1.0,  1.0,  0.0),
    'e': ( 1.0, -1.0,  0.0),
    'z': (-1.0,  1.0,  0.0),
    'c': (-1.0, -1.0,  0.0),
    'j': ( 0.0,  0.0,  1.0),
    'l': ( 0.0,  0.0, -1.0),
    's': ( 0.0,  0.0,  0.0),
    ' ': ( 0.0,  0.0,  0.0),
}

YARDIM = """
╔═══════════════════════════════════════════════════════════╗
║          MECANUM TELEOP v2 — Klavye Kontrol               ║
╠═══════════════════════════════════════════════════════════╣
║   Q   W   E      ↖    ↑    ↗                              ║
║   A   S   D      ←   DUR   →     (A/D = yana kayma)       ║
║   Z   X   C      ↙    ↓    ↘                              ║
║                                                           ║
║   J / L   yavas don (sol/sag)                             ║
║   U / O   TAM 90° don (sol/sag) — otomatik, birak calissin║
║                                                           ║
║   1 / 2   lineer hiz  -/+        3 / 4   acisal hiz  -/+  ║
║   F       hassas mod (hizlar yariya)                      ║
║   S/BOSLUK  DUR                  CTRL-C  cikis            ║
╚═══════════════════════════════════════════════════════════╝
"""


class MecanumTeleop(Node):

    def __init__(self):
        super().__init__('mecanum_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel_teleop', 10)

        # SLAM icin uygun baslangic degerleri
        self.linear_speed = 0.25     # m/s
        self.angular_speed = 0.35    # rad/s — v1'de 0.5'ti, donus zordu
        self.hassas_mod = False

        self.get_logger().info('Mecanum teleop v2 hazir.')

    def carpan(self):
        """Hassas mod aciksa hizlari yariya indir."""
        return 0.5 if self.hassas_mod else 1.0

    def hiz_yayinla(self, x, y, z):
        k = self.carpan()
        msg = Twist()
        msg.linear.x = x * self.linear_speed * k
        msg.linear.y = y * self.linear_speed * k
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = z * self.angular_speed * k
        self.publisher.publish(msg)

    def dur(self):
        self.hiz_yayinla(0.0, 0.0, 0.0)

    def doksan_derece_don(self, yon):
        """
        Tam 90 derece donus yap (yon: +1 sol, -1 sag).

        Nasil calisir: sabit acisal hizda, 90 derece icin gereken
        sure kadar donup durur. Acik dongu (open-loop) bir yaklasim —
        odometri geri beslemesi kullanmaz, ama simulasyonda yeterince
        tutarli. SLAM icin cok degerli: elle donuste aci tutturmak
        zordur, bu tam 90 derece verir.
        """
        hiz = 0.5   # rad/s — sabit, kontrollu donus hizi
        sure = (math.pi / 2) / hiz   # 90 derece / hiz = gereken saniye

        baslangic = time.time()
        while time.time() - baslangic < sure:
            self.hiz_yayinla(0.0, 0.0, yon * hiz / self.angular_speed
                             / self.carpan())
            time.sleep(0.02)
        self.dur()

    def durum_yaz(self):
        mod = ' [HASSAS]' if self.hassas_mod else ''
        k = self.carpan()
        print(f'\r  lineer: {self.linear_speed*k:.2f} m/s   '
              f'acisal: {self.angular_speed*k:.2f} rad/s{mod}     ',
              end='', flush=True)


def tus_oku(timeout=0.1):
    fd = sys.stdin.fileno()
    eski = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        hazir, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if hazir else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, eski)


def stdin_tty_mi():
    """
    stdin gercek bir terminale bagli mi?

    'ros2 launch' alt surecin stdin'ini kendi surec yonetimine baglar
    (emulate_tty=True yalniz CIKTI icin pty acar). O durumda termios
    cagrilari ENOTTY ('Inappropriate ioctl for device') ile patlar.
    Bu yuzden node ayri, interaktif bir terminalden 'ros2 run' ile
    calistirilmalidir.
    """
    try:
        return os.isatty(sys.stdin.fileno())
    except (ValueError, OSError):
        return False


def main(args=None):
    if not stdin_tty_mi():
        print(
            '\nHATA: teleop_node interaktif bir terminale bagli degil '
            '(stdin tty degil).\n'
            '  ros2 launch ile calistirmayin — launch stdin\'i devralir.\n'
            '  Bunun yerine kendi terminalinizde:\n\n'
            '      ros2 run mecanum_teleop teleop_node\n',
            file=sys.stderr)
        return 1

    rclpy.init(args=args)
    node = MecanumTeleop()

    print(YARDIM)
    node.durum_yaz()

    try:
        while rclpy.ok():
            tus = tus_oku()

            if tus == '\x03':
                break

            elif tus in TUS_HARITASI:
                x, y, z = TUS_HARITASI[tus]
                node.hiz_yayinla(x, y, z)

            elif tus == 'u':
                print('\r  90° sola donuluyor...                    ',
                      end='', flush=True)
                node.doksan_derece_don(+1)
                node.durum_yaz()

            elif tus == 'o':
                print('\r  90° saga donuluyor...                    ',
                      end='', flush=True)
                node.doksan_derece_don(-1)
                node.durum_yaz()

            elif tus == '1':
                node.linear_speed = max(0.05, node.linear_speed - 0.05)
                node.durum_yaz()
            elif tus == '2':
                node.linear_speed = min(1.0, node.linear_speed + 0.05)
                node.durum_yaz()
            elif tus == '3':
                node.angular_speed = max(0.1, node.angular_speed - 0.05)
                node.durum_yaz()
            elif tus == '4':
                node.angular_speed = min(2.0, node.angular_speed + 0.05)
                node.durum_yaz()

            elif tus == 'f':
                node.hassas_mod = not node.hassas_mod
                node.durum_yaz()

            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f'\nHata: {e}')

    finally:
        node.dur()
        node.destroy_node()
        rclpy.shutdown()
        print('\n\nTeleop kapatildi, robot durduruldu.')

    return 0


if __name__ == '__main__':
    sys.exit(main())
