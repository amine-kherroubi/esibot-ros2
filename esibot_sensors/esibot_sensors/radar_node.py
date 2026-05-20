#!/usr/bin/env python3
"""
EsibotSensors - HC-SR04 sweep node
==================================
Architecture réelle (wiring diagram v3) :
  - RPi GPIO12 → 1kΩ → MG996R signal  (pigpio hardware DMA PWM 50Hz)
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

# ── GPIO (RPi : TRIG + ECHO via RPi.GPIO, servo via pigpio) ─────────────────
try:
    import RPi.GPIO as GPIO
    HARDWARE_AVAILABLE = True
    GPIO_IMPORT_ERROR = None
except ImportError as exc:
    GPIO = None
    HARDWARE_AVAILABLE = False
    GPIO_IMPORT_ERROR = exc

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    pigpio = None
    PIGPIO_AVAILABLE = False

GPIO_SERVO_PIN = 12   # RPi Pin 32 — pigpio hardware DMA PWM 50Hz, 1kΩ series resistor


class EsibotSensors(Node):
    def __init__(self):
        super().__init__("radar_node")
        self.log = get_logger(node=self)

        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)

        # ── Géométrie du scan ────────────────────────────────────────────────
        self.angle_min = -math.radians(100)  # -100° → droite
        self.angle_max =  math.radians(100)  # +100° → gauche
        self.angle_increment = math.radians(10)  # 10°/step → 21 mesures (beam HC-SR04 ≈ 30°)

        # ── Plage valide HC-SR04 (datasheet) ────────────────────────────────
        self.range_min = 0.02   # 2 cm
        self.range_max = 4.00   # 400 cm

        # ── Paramètres ROS ───────────────────────────────────────────────────
        self.declare_parameter("trig_pin",      23)   # RPi Pin 16 → HC-SR04 TRIG
        self.declare_parameter("echo_pin",      25)   # RPi Pin 22 → diviseur → HC-SR04 ECHO
        self.declare_parameter("sweep_period",   8.0) # 21 steps × ~325ms ≈ 7s
        self.declare_parameter("median_reads",  3)    # lectures par angle, retourne la médiane
        self.declare_parameter("sim_mode",      False)

        self.trig_pin        = self.get_parameter("trig_pin").value
        self.echo_pin        = self.get_parameter("echo_pin").value
        self._sweep_period   = float(self.get_parameter("sweep_period").value)
        self._median_reads   = int(self.get_parameter("median_reads").value)

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

        # ── Servo sur GPIO12 — pigpio hardware DMA PWM (fallback RPi.GPIO) ────
        self._pi         = None   # pigpio instance
        self._servo_pwm  = None   # RPi.GPIO fallback
        self._current_pulse_us = 1500  # track position for smooth ramp

        if HARDWARE_AVAILABLE:
            if PIGPIO_AVAILABLE:
                try:
                    self._pi = pigpio.pi()
                    if not self._pi.connected:
                        self._pi = None
                        self.log.error(
                            "pigpiod non démarré — sudo systemctl start pigpiod. "
                            "Fallback vers RPi.GPIO software PWM."
                        )
                    else:
                        self._pi.set_servo_pulsewidth(GPIO_SERVO_PIN, 1500)
                        self.log.info(
                            f"pigpio servo initialisé sur GPIO{GPIO_SERVO_PIN} "
                            "(hardware DMA PWM — jitter < 1µs)"
                        )
                except Exception as exc:
                    self._pi = None
                    self.log.error(f"Erreur init pigpio : {exc}")

            if self._pi is None:
                # Fallback : RPi.GPIO software PWM
                try:
                    GPIO.setup(GPIO_SERVO_PIN, GPIO.OUT)
                    self._servo_pwm = GPIO.PWM(GPIO_SERVO_PIN, 50)
                    self._servo_pwm.start(7.5)
                    self.log.warning(
                        f"Servo fallback RPi.GPIO software PWM sur GPIO{GPIO_SERVO_PIN} "
                        "(jitter élevé — installer pigpio pour un sweep fluide)"
                    )
                except Exception as exc:
                    self._servo_pwm = None
                    self.log.error(f"Erreur init servo fallback : {exc}")

        # ── Ping-pong sweep direction (+1 = min→max, -1 = max→min) ─────────────
        self._sweep_dir = 1

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

        # Build angle list then order by sweep direction (ping-pong)
        angles = []
        a = self.angle_min
        while a <= self.angle_max + 1e-9:
            angles.append(a)
            a += self.angle_increment
        if self._sweep_dir < 0:
            angles = list(reversed(angles))

        ranges_ordered = []
        for angle in angles:
            js            = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name       = ["servo_joint"]
            js.position   = [angle]
            self._joint_state_pub.publish(js)

            raw = self.read_distance(angle)
            dist = raw if self.range_min <= raw <= self.range_max else float('inf')
            ranges_ordered.append(dist)

        # LaserScan always published min→max; reverse if backward sweep
        ranges = list(reversed(ranges_ordered)) if self._sweep_dir < 0 else ranges_ordered
        self._sweep_dir *= -1  # flip for next sweep

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
        1. Commande le servo MG996R via PWM GPIO12.
        2. Attend 120ms de stabilisation (pas de 5° → course réduite).
        3. Prend self._median_reads lectures HC-SR04, retourne la médiane.
           Entre lectures consécutives 20ms de pause (évite les échos croisés).
        """
        if HARDWARE_AVAILABLE:
            t_send = time.time()
            self._set_servo_angle(angle)  # pigpio: inclut 50ms de rampe interne
            elapsed = time.time() - t_send
            remaining = max(0.0, 0.10 - elapsed)  # 100ms total settle (ramp ~50ms + 50ms extra)
            if remaining > 0:
                time.sleep(remaining)

            samples = []
            for i in range(self._median_reads):
                if i > 0:
                    time.sleep(0.020)
                d = self.hc_sr04_distance()
                if not math.isinf(d):
                    samples.append(d)

            if not samples:
                return float('inf')
            samples.sort()
            return samples[len(samples) // 2]
        else:
            time.sleep(0.01)
            return 1.0 + random.gauss(0.0, 0.02)

    # ── Servo (GPIO12) — pigpio hardware PWM avec rampe douce ───────────────

    def _set_servo_angle(self, angle: float):
        """
        Déplace le servo vers `angle` avec une rampe en 5 étapes (50ms total).
        Utilise pigpio DMA PWM si disponible, sinon fallback RPi.GPIO (sans rampe).
        """
        angle_deg = math.degrees(angle)
        target_us = int(max(900.0, min(2100.0, 1500.0 + angle_deg * (600.0 / 100.0))))

        if self._pi is not None:
            # Rampe fluide : 5 étapes × 10ms = 50ms, interpolation linéaire
            start_us = self._current_pulse_us
            for i in range(1, 6):
                us = int(start_us + (target_us - start_us) * i / 5)
                self._pi.set_servo_pulsewidth(GPIO_SERVO_PIN, us)
                time.sleep(0.010)
            self._current_pulse_us = target_us
            self.log.debug(f"Servo pigpio : {angle_deg:.1f}° → {target_us}µs")
        elif self._servo_pwm is not None:
            duty_cycle = target_us / 20000.0 * 100.0
            try:
                self._servo_pwm.ChangeDutyCycle(duty_cycle)
                self._current_pulse_us = target_us
            except Exception as exc:
                self.log.error(f"Erreur servo fallback : {exc}")

    # ── HC-SR04 (branché directement sur le RPi) ─────────────────────────────

    def hc_sr04_distance(self) -> float:
        """
        TRIG : GPIO23 (RPi Pin 16) — sortie 3.3V, suffisant pour déclencher.
        ECHO : GPIO25 (RPi Pin 22) via diviseur R1=1.2kΩ / R2=1.5kΩ → 2.78V ✓
        Retourne la distance en mètres, ou range_max + 1.0 en cas de timeout.
        """
        # Reset TRIG — 60ms recommended by HC-SR04 datasheet between measurements
        GPIO.output(self.trig_pin, False)
        time.sleep(0.06)

        # Impulsion TRIG 10 µs
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        # Attente front montant ECHO (rising edge)
        timeout = time.time() + 0.10
        while GPIO.input(self.echo_pin) == 0:
            if time.time() > timeout:
                self.log.warning("HC-SR04 : timeout front montant ECHO")
                return float('inf')
        start = time.time()

        # Attente front descendant ECHO (falling edge) — 200ms for cheap clones
        timeout = time.time() + 0.20
        while GPIO.input(self.echo_pin) == 1:
            if time.time() > timeout:
                self.log.warning("HC-SR04 : timeout front descendant ECHO")
                return float('inf')
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
            if node._pi is not None:
                try:
                    node._pi.set_servo_pulsewidth(GPIO_SERVO_PIN, 0)
                    node._pi.stop()
                except Exception:
                    pass
            elif node._servo_pwm is not None:
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
