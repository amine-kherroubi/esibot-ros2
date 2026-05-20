#!/usr/bin/env python3
"""
EsibotSensors - HC-SR04 sweep node
==================================
Architecture réelle (wiring diagram v3) :
  - RPi GPIO12 → 1kΩ → MG996R signal  (pigpio hardware DMA PWM 50Hz)
  - RPi GPIO23 → HC-SR04 TRIG
  - RPi GPIO25 ← HC-SR04 ECHO (via diviseur R1=1.2kΩ/R2=1.5kΩ → 2.78V)
  - RPi GPIO24 ← right wheel encoder D0 (moved here from GPIO18)

HC-SR04 timing: pigpio hardware callbacks (1µs precision) preferred over
RPi.GPIO busy-wait (~500µs jitter on Linux). All GPIO managed by pigpio
when pigpiod is running; RPi.GPIO used as fallback only.
"""

import math
import random
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState

from esibot_logging import get_logger, setup_logging

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

        # ── Scan geometry ────────────────────────────────────────────────────
        self.angle_min = -math.radians(90)   # -90° (robot body clips beyond this)
        self.angle_max =  math.radians(90)   # +90°
        self.angle_increment = math.radians(10)  # 10°/step → 19 beams

        # ── HC-SR04 valid range ──────────────────────────────────────────────
        # range_min=0.20 filters robot self-reflections (chassis/wheels at ~7-17cm)
        # and voltage-divider noise. Datasheet min is 2cm but robot body occupies
        # the inner ~18cm at lateral angles.
        self.range_min = 0.20   # 20 cm — below this is robot body or noise
        self.range_max = 4.00   # 400 cm

        # ── ROS parameters ───────────────────────────────────────────────────
        self.declare_parameter("trig_pin",      23)
        self.declare_parameter("echo_pin",      25)
        self.declare_parameter("sweep_period",   6.0)
        self.declare_parameter("median_reads",  1)
        self.declare_parameter("sim_mode",      False)

        self.trig_pin        = self.get_parameter("trig_pin").value
        self.echo_pin        = self.get_parameter("echo_pin").value
        self._sweep_period   = float(self.get_parameter("sweep_period").value)
        self._median_reads   = int(self.get_parameter("median_reads").value)

        # ── HC-SR04 pigpio callback state ────────────────────────────────────
        self._echo_cb         = None          # pigpio.callback handle
        self._echo_start_tick = None          # pigpio tick at rising ECHO edge
        self._echo_ready      = threading.Event()
        self._echo_distance   = float('inf')

        # ── Servo state ──────────────────────────────────────────────────────
        self._pi             = None
        self._servo_pwm      = None
        self._current_pulse_us = 1500

        # ── GPIO init: pigpio for everything when available ──────────────────
        if HARDWARE_AVAILABLE:
            if PIGPIO_AVAILABLE:
                try:
                    self._pi = pigpio.pi()
                    if not self._pi.connected:
                        self._pi = None
                        self.log.error(
                            "pigpiod not running — sudo systemctl start pigpiod. "
                            "Falling back to RPi.GPIO."
                        )
                    else:
                        # TRIG as output, ECHO as input with hardware callback
                        self._pi.set_mode(self.trig_pin, pigpio.OUTPUT)
                        self._pi.write(self.trig_pin, 0)
                        self._pi.set_mode(self.echo_pin, pigpio.INPUT)
                        self._pi.set_pull_up_down(self.echo_pin, pigpio.PUD_DOWN)
                        self._echo_cb = self._pi.callback(
                            self.echo_pin, pigpio.EITHER_EDGE, self._pigpio_echo_cb
                        )
                        # Servo center
                        self._pi.set_servo_pulsewidth(GPIO_SERVO_PIN, 1500)
                        self.log.info(
                            f"pigpio: servo=GPIO{GPIO_SERVO_PIN}, "
                            f"TRIG=GPIO{self.trig_pin}, ECHO=GPIO{self.echo_pin} "
                            "(hardware callbacks — jitter <1µs)"
                        )
                except Exception as exc:
                    self._pi = None
                    self.log.error(f"pigpio init error: {exc}")

            if self._pi is None:
                # Fallback: RPi.GPIO for TRIG+ECHO+servo (busy-wait timing)
                try:
                    GPIO.setmode(GPIO.BCM)
                    GPIO.setup(self.trig_pin, GPIO.OUT)
                    GPIO.setup(self.echo_pin, GPIO.IN)
                    GPIO.output(self.trig_pin, False)
                    GPIO.setup(GPIO_SERVO_PIN, GPIO.OUT)
                    self._servo_pwm = GPIO.PWM(GPIO_SERVO_PIN, 50)
                    self._servo_pwm.start(7.5)
                    self.log.warning(
                        f"RPi.GPIO fallback — TRIG=GPIO{self.trig_pin}, "
                        f"ECHO=GPIO{self.echo_pin}, servo=GPIO{GPIO_SERVO_PIN} "
                        "(busy-wait timing, ~500µs jitter)"
                    )
                except Exception as exc:
                    self.log.error(f"RPi.GPIO init error: {exc}")
        else:
            self.log.warning(
                f"RPi.GPIO not found — SIMULATION mode active. ({GPIO_IMPORT_ERROR})"
            )

        # ── Ping-pong sweep direction ────────────────────────────────────────
        self._sweep_dir = 1

        # ── Re-entrance guard ────────────────────────────────────────────────
        self._scanning = False

        # ── Timers ───────────────────────────────────────────────────────────
        self.timer = self.create_timer(self._sweep_period, self.timer_callback)
        # 10 Hz JointState keeps servo_link→laser_link TF alive for SLAM
        self._js_timer = self.create_timer(0.1, self._publish_servo_neutral)

    # ── pigpio ECHO callback (runs in pigpiod thread) ────────────────────────

    def _pigpio_echo_cb(self, gpio, level, tick):
        if level == 1:
            self._echo_start_tick = tick
        elif level == 0 and self._echo_start_tick is not None:
            elapsed_us = pigpio.tickDiff(self._echo_start_tick, tick)
            self._echo_distance = elapsed_us * 343.0 / 2.0 / 1e6
            self._echo_start_tick = None
            self._echo_ready.set()

    # ── Keep laser_link TF alive ─────────────────────────────────────────────

    def _publish_servo_neutral(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["servo_joint"]
        js.position = [0.0]
        self._joint_state_pub.publish(js)

    # ── Timer callback ───────────────────────────────────────────────────────

    def timer_callback(self):
        if self._scanning:
            self.log.warning(
                "Previous sweep still running — tick skipped. "
                "Increase sweep_period if this is frequent."
            )
            return
        self.publish_scan()

    # ── Scan publication ─────────────────────────────────────────────────────

    def publish_scan(self):
        self._scanning = True
        sweep_start = time.time()

        msg = LaserScan()
        msg.header.frame_id = "laser_link"
        msg.angle_min       = self.angle_min
        msg.angle_max       = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment  = 0.0
        msg.range_min       = self.range_min
        msg.range_max       = self.range_max

        angles = []
        a = self.angle_min
        while a <= self.angle_max + 1e-9:
            angles.append(a)
            a += self.angle_increment
        if self._sweep_dir < 0:
            angles = list(reversed(angles))

        ranges_ordered = []
        for angle in angles:
            raw = self.read_distance(angle)
            dist = raw if self.range_min <= raw <= self.range_max else float('inf')
            ranges_ordered.append(dist)

        ranges = list(reversed(ranges_ordered)) if self._sweep_dir < 0 else ranges_ordered
        self._sweep_dir *= -1

        sweep_duration      = time.time() - sweep_start
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.scan_time       = float(sweep_duration)
        msg.ranges          = ranges
        msg.time_increment  = float(sweep_duration) / max(len(ranges) - 1, 1)

        valid_count = sum(1 for r in ranges if math.isfinite(r))
        if valid_count == 0:
            self.log.warning(
                f"Invalid scan — 0/{len(ranges)} valid readings (sensor disconnected?). "
                "Scan not published."
            )
            self._scanning = False
            return

        self._scan_pub.publish(msg)
        self.log.info(
            f"LaserScan published: {valid_count}/{len(ranges)} valid, "
            f"sweep_time={sweep_duration:.3f}s"
        )
        self._scanning = False

    # ── Hardware / simulation abstraction ────────────────────────────────────

    def read_distance(self, angle: float) -> float:
        if HARDWARE_AVAILABLE:
            t_send = time.time()
            self._set_servo_angle(angle)
            elapsed = time.time() - t_send
            remaining = max(0.0, 0.10 - elapsed)
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

    # ── Servo (GPIO12) — smooth ramp via pigpio DMA PWM ──────────────────────

    def _set_servo_angle(self, angle: float):
        angle_deg = math.degrees(angle)
        target_us = int(max(900.0, min(2100.0, 1500.0 + angle_deg * (600.0 / 100.0))))

        if self._pi is not None:
            start_us = self._current_pulse_us
            for i in range(1, 6):
                us = int(start_us + (target_us - start_us) * i / 5)
                self._pi.set_servo_pulsewidth(GPIO_SERVO_PIN, us)
                time.sleep(0.010)
            self._current_pulse_us = target_us
            self.log.debug(f"Servo pigpio: {angle_deg:.1f}° → {target_us}µs")
        elif self._servo_pwm is not None:
            duty_cycle = target_us / 20000.0 * 100.0
            try:
                self._servo_pwm.ChangeDutyCycle(duty_cycle)
                self._current_pulse_us = target_us
            except Exception as exc:
                self.log.error(f"Servo fallback error: {exc}")

    # ── HC-SR04 distance measurement ─────────────────────────────────────────

    def hc_sr04_distance(self) -> float:
        if self._pi is not None:
            return self._hc_sr04_pigpio()
        return self._hc_sr04_gpio()

    def _hc_sr04_pigpio(self) -> float:
        """Microsecond-accurate measurement via pigpio hardware callback timestamps."""
        self._echo_ready.clear()
        self._echo_start_tick = None
        self._echo_distance = float('inf')

        # HC-SR04 protocol: settle 60ms, trigger 10µs pulse
        self._pi.write(self.trig_pin, 0)
        time.sleep(0.06)
        self._pi.write(self.trig_pin, 1)
        time.sleep(0.00001)
        self._pi.write(self.trig_pin, 0)

        if self._echo_ready.wait(timeout=0.20):
            return self._echo_distance
        self.log.warning("HC-SR04 pigpio: echo timeout (sensor disconnected?)")
        return float('inf')

    def _hc_sr04_gpio(self) -> float:
        """Fallback: RPi.GPIO busy-wait (~500µs timing jitter on Linux)."""
        GPIO.output(self.trig_pin, False)
        time.sleep(0.06)
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)

        timeout = time.time() + 0.10
        while GPIO.input(self.echo_pin) == 0:
            if time.time() > timeout:
                self.log.warning("HC-SR04 GPIO: rising edge timeout")
                return float('inf')
        start = time.time()

        timeout = time.time() + 0.20
        while GPIO.input(self.echo_pin) == 1:
            if time.time() > timeout:
                self.log.warning("HC-SR04 GPIO: falling edge timeout")
                return float('inf')
        end = time.time()

        return (end - start) * 343.0 / 2.0


def main(args=None):
    setup_logging()
    rclpy.init(args=args)
    node = EsibotSensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._echo_cb is not None:
            try:
                node._echo_cb.cancel()
            except Exception:
                pass
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
                    GPIO.cleanup()
                except Exception:
                    pass
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
