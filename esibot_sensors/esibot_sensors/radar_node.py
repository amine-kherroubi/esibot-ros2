#!/usr/bin/env python3
"""
EsibotSensors - HC-SR04 sweep node
==================================
Architecture réelle (wiring diagram v3) :
  - RPi GPIO12 → 1kΩ → MG996R signal  (software PWM 50Hz, RPi.GPIO)
  - RPi GPIO23 → HC-SR04 TRIG
  - RPi GPIO25 ← HC-SR04 ECHO (via diviseur R1=1.2kΩ/R2=1.5kΩ → 2.78V)
  - RPi GPIO24 ← right wheel encoder D0 (moved here from GPIO18)
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState

from esibot_logging import get_logger, setup_logging

# ── GPIO (RPi : TRIG + ECHO) ─────────────────────────────────────────────────
try:
    import RPi.GPIO as GPIO
    HARDWARE_AVAILABLE = True
    GPIO_IMPORT_ERROR = None
except ImportError as exc:
    GPIO = None
    HARDWARE_AVAILABLE = False
    GPIO_IMPORT_ERROR = exc

GPIO_SERVO_PIN = 12   # RPi Pin 32 — software PWM 50Hz, 1kΩ series resistor


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
        self.declare_parameter("trig_pin",     23)   # RPi Pin 16 → HC-SR04 TRIG
        self.declare_parameter("echo_pin",     25)   # RPi Pin 22 → diviseur → HC-SR04 ECHO
        self.declare_parameter("sweep_period",  3.0)
        self.declare_parameter("sim_mode",     False)

        self.trig_pin      = self.get_parameter("trig_pin").value
        self.echo_pin      = self.get_parameter("echo_pin").value
        self._sweep_period = float(self.get_parameter("sweep_period").value)

        # ── Init GPIO RPi (TRIG + ECHO seulement) ───────────────────────────
        if HARDWARE_AVAILABLE:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.trig_pin, GPIO.OUT)
            GPIO.setup(self.echo_pin, GPIO.IN)
            GPIO.output(self.trig_pin, False)
            self.log.info(
                f"GPIO initialisé — TRIG=GPIO{self.trig_pin}, "
                f"ECHO=GPIO{self.echo_pin} (via diviseur R1=1.2kΩ/R2=1.5kΩ → 2.78V)"
            )
        else:
            self.log.warning(
                f"RPi.GPIO introuvable — mode SIMULATION actif. ({GPIO_IMPORT_ERROR})"
            )

        # ── Servo PWM sur GPIO12 (RPi.GPIO software PWM 50Hz) ───────────────
        self._servo_pwm = None
        if HARDWARE_AVAILABLE:
            try:
                GPIO.setup(GPIO_SERVO_PIN, GPIO.OUT)
                self._servo_pwm = GPIO.PWM(GPIO_SERVO_PIN, 50)  # 50Hz
                self._servo_pwm.start(7.5)  # centre (1500µs / 20ms = 7.5%)
                self.log.info(f"Servo PWM initialisé sur GPIO{GPIO_SERVO_PIN} (50Hz)")
            except Exception as exc:
                self._servo_pwm = None
                self.log.error(f"Erreur init servo PWM : {exc}")

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
        1. Commande le servo MG996R via PWM hardware GPIO12 (pigpio).
        2. Attend la stabilisation (200ms garantis depuis l'envoi).
        3. Déclenche le HC-SR04 et retourne la distance (m).
        """
        if HARDWARE_AVAILABLE:
            t_send = time.time()
            self._set_servo_angle(angle)
            elapsed = time.time() - t_send
            remaining = max(0.0, 0.20 - elapsed)
            if remaining > 0:
                time.sleep(remaining)
            return self.hc_sr04_distance()
        else:
            time.sleep(0.01)
            return 1.0 + random.gauss(0.0, 0.02)

    # ── Servo PWM direct (GPIO12, RPi.GPIO 50Hz) ─────────────────────────────

    def _set_servo_angle(self, angle: float):
        """Convertit l'angle ROS [-π/2, +π/2] en duty cycle 50Hz et l'applique."""
        if self._servo_pwm is None:
            return
        angle_deg  = math.degrees(angle)                      # -90 à +90
        pulse_us   = 1500.0 + angle_deg * (500.0 / 90.0)     # 1000–2000 µs
        pulse_us   = max(1000.0, min(2000.0, pulse_us))
        duty_cycle = pulse_us / 20000.0 * 100.0               # 5.0–10.0 %
        try:
            self._servo_pwm.ChangeDutyCycle(duty_cycle)
            self.log.debug(f"Servo : {angle_deg:.1f}° → {pulse_us:.0f}µs ({duty_cycle:.2f}%)")
        except Exception as exc:
            self.log.error(f"Erreur servo PWM : {exc}")

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
            if node._servo_pwm is not None:
                try:
                    node._servo_pwm.stop()
                except Exception:
                    pass
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
