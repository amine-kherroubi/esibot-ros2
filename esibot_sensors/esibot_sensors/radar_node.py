#!/usr/bin/env python3
"""
EsibotSensors - HC-SR04 sweep node
==================================
Architecture réelle (wiring diagram) :
  - RPi      → HC-SR04  : TRIG=GPIO23, ECHO=GPIO24 (via diviseur 1.2kΩ/1.5kΩ)
  - ESP32-CAM → MG996R  : PWM sur GPIO15 (50Hz, pull-down 10kΩ hardware obligatoire)
  - RPi ↔ ESP32-CAM     : UART (RPi TX=GPIO14/Pin8, RX=GPIO15/Pin10)

Le RPi envoie la consigne d'angle au ESP32 via UART,
attend que le servo soit en position, puis déclenche le HC-SR04.
"""

import math
import random
import serial          # pyserial — communication UART vers ESP32
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState

from esibot_logging import get_logger, setup_logging

# ── GPIO (RPi uniquement : TRIG + ECHO) ─────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    HARDWARE_AVAILABLE = True
    GPIO_IMPORT_ERROR = None
except ImportError as exc:
    GPIO = None
    HARDWARE_AVAILABLE = False
    GPIO_IMPORT_ERROR = exc


class EsibotSensors(Node):
    def __init__(self):
        super().__init__("radar_node")
        self.log = get_logger(node=self)

        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # ── Géométrie du scan ────────────────────────────────────────────────
        self.angle_min = -math.pi / 2   # -90° → droite
        self.angle_max =  math.pi / 2   # +90° → gauche
        self.angle_increment = math.radians(10)  # 10°/step → 19 mesures

        # ── Plage valide HC-SR04 (datasheet) ────────────────────────────────
        self.range_min = 0.02   # 2 cm
        self.range_max = 4.00   # 400 cm

        # ── Paramètres ROS ───────────────────────────────────────────────────
        # Pins RPi (BCM) — conformes au wiring diagram
        self.declare_parameter("trig_pin",     23)   # RPi Pin 16 → HC-SR04 TRIG
        self.declare_parameter("echo_pin",     24)   # RPi Pin 18 → diviseur → HC-SR04 ECHO
        self.declare_parameter("sweep_period",  3.0)
        self.declare_parameter("sim_mode",     False)

        # Port UART vers ESP32-CAM
        self.declare_parameter("uart_port",  "/dev/ttyS0")
        self.declare_parameter("uart_baud",   115200)

        self.trig_pin     = self.get_parameter("trig_pin").value
        self.echo_pin     = self.get_parameter("echo_pin").value
        self._sweep_period = float(self.get_parameter("sweep_period").value)
        uart_port         = self.get_parameter("uart_port").value
        uart_baud         = self.get_parameter("uart_baud").value

        # ── Init GPIO RPi (TRIG + ECHO seulement) ───────────────────────────
        if HARDWARE_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trig_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trig_pin, False)
            self.log.info(
                f"GPIO initialisé — TRIG=GPIO{self.trig_pin}, "
                f"ECHO=GPIO{self.echo_pin} (via diviseur 1.2kΩ/1.5kΩ)"
            )
        else:
            self.log.warning(
                f"RPi.GPIO introuvable — mode SIMULATION actif. ({GPIO_IMPORT_ERROR})"
            )

        # ── UART vers ESP32-CAM (contrôle MG996R) ───────────────────────────
        self._uart = None
        if HARDWARE_AVAILABLE:
            try:
                self._uart = serial.Serial(uart_port, uart_baud, timeout=1.0, write_timeout=1.0)
                    # Flush des buffers UART
                self._uart.reset_input_buffer()
                self._uart.reset_output_buffer()
                self.log.info(f"UART ouvert : {uart_port} @ {uart_baud} baud")
            except Exception as exc:
                self.log.error(
                    f"Impossible d'ouvrir le port UART {uart_port} : {exc}. "
                    "Le servo ne bougera pas."
                )

        # ── Garde anti-réentrance ────────────────────────────────────────────
        self._scanning = False

        # ── Timer ────────────────────────────────────────────────────────────
        self.timer = self.create_timer(self._sweep_period, self.timer_callback)

    # ── Timer callback ───────────────────────────────────────────────────────

    def timer_callback(self):
        if self._scanning:
            self.log.warning(
                "Sweep précédent encore en cours — tick ignoré. "
                "Augmentez sweep_period si ce message est fréquent."
            )
            return
        self.publish_scan()

    # ── Publication du scan ──────────────────────────────────────────────────

    def publish_scan(self):
        self._scanning = True
        sweep_start = time.time()

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        msg.angle_min       = self.angle_min
        msg.angle_max       = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment  = 0.0
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max

        ranges = []
        angle  = self.angle_min

        while angle <= self.angle_max + 1e-9:

            # Publie la position articulaire (pour RViz / URDF)
            js            = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name       = ["servo_joint"]
            js.position   = [angle]
            self._joint_state_pub.publish(js)

            raw = self.read_distance(angle)

            dist = raw if self.range_min <= raw <= self.range_max else self.range_max + 1.0
            ranges.append(dist)
            angle += self.angle_increment

        sweep_duration      = time.time() - sweep_start
        msg.scan_time       = float(sweep_duration)
        msg.ranges          = ranges
        msg.time_increment  = float(sweep_duration) / max(len(ranges) - 1, 1)

        self._scan_pub.publish(msg)
        self.log.info(
            f"LaserScan publié : {len(ranges)} mesures, "
            f"sweep_time={sweep_duration:.3f}s, "
            f"ranges={[f'{r:.2f}' for r in ranges]}"
        )
        self._scanning = False

    # ── Abstraction hardware / simulation ────────────────────────────────────

    def read_distance(self, angle: float) -> float:
        """
        1. Envoie la consigne d'angle au ESP32 via UART → ESP32 pilote le MG996R.
        2. Attend la stabilisation du servo (100 ms).
        3. Déclenche le HC-SR04 depuis le RPi et retourne la distance (m).
        """
        if HARDWARE_AVAILABLE:
            self._send_servo_angle(angle)   # ESP32 bouge le MG996R
            time.sleep(0.20)                # attente stabilisation (10° step)
            return self.hc_sr04_distance()
        else:
            time.sleep(0.01)
            return 1.0 + random.gauss(0.0, 0.02)

    # ── Commande servo via UART (MG996R piloté par ESP32-CAM GPIO15) ─────────

    def _send_servo_angle(self, angle: float):
        """
        Envoie une commande texte simple au ESP32 :
          "SERVO <angle_deg>\n"
        Exemples : "SERVO -90\n", "SERVO 0\n", "SERVO 45\n"

        Le firmware ESP32 doit parser cette commande et générer le PWM
        sur GPIO15 (50 Hz, 1–2 ms pulse width, pull-down 10kΩ hardware).
        """
        if self._uart is None:
            self.log.warning("UART non disponible — servo non commandé.")
            return

        angle_deg = round(math.degrees(angle))
        cmd       = f"SERVO:{angle_deg}\n"
        try:
            self._uart.write(cmd.encode("ascii"))
            self.log.debug(f"UART → ESP32 : {cmd.strip()}")
            ack = self._uart.readline().decode().strip()

            if ack != "OK":
              self.log.warning(f"ACK invalide ESP32 : '{ack}'") 
        except Exception as exc:
            self.log.error(f"Erreur écriture UART : {exc}")

    # ── HC-SR04 (branché directement sur le RPi) ─────────────────────────────

    def hc_sr04_distance(self) -> float:
        """
        TRIG : GPIO23 (RPi Pin 16) — sortie 3.3V, suffisant pour déclencher.
        ECHO : GPIO24 (RPi Pin 18) via diviseur R1=1.2kΩ / R2=1.5kΩ → 2.78V ✓
        Retourne la distance en mètres, ou range_max + 1.0 en cas de timeout.
        """
        # Reset TRIG
        GPIO.output(self.trig_pin, False)
        time.sleep(0.01)

        # Impulsion TRIG 10 µs
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        # Attente front montant ECHO (rising edge)
        timeout = time.time() + 0.05
        while GPIO.input(self.echo_pin) == 0:
            if time.time() > timeout:
                self.log.warning("HC-SR04 : timeout front montant ECHO")
                return self.range_max + 1.0
        start = time.time()

        # Attente front descendant ECHO (falling edge)
        timeout = time.time() + 0.05
        while GPIO.input(self.echo_pin) == 1:
            if time.time() > timeout:
                self.log.warning("HC-SR04 : timeout front descendant ECHO")
                return self.range_max + 1.0
        end = time.time()

        # Distance = (durée × vitesse_son) / 2  [343 m/s à 20°C]
        return ((end - start) * 343.0) / 2.0


def main(args=None):
    setup_logging()
    rclpy.init(args=args)
    node = EsibotSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if HARDWARE_AVAILABLE:
            if node._uart:
                node._uart.close()
            GPIO.cleanup()
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
