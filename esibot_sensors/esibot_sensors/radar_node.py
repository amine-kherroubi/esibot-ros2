#!/usr/bin/env python3
"""
HC-SR04 sweep node
==================
Wiring:
  RPi GPIO12 → 1kΩ → MG996R signal  (pigpio DMA PWM 50 Hz)
  RPi GPIO23 → HC-SR04 TRIG
  RPi GPIO25 ← HC-SR04 ECHO (voltage divider → 2.78 V)

Servo convention (physical mounting):
  servo_angle = +180° → pulse MAX (2500µs) → sensor points RIGHT extreme
  servo_angle =    0° → pulse 1500µs       → sensor points FORWARD
  servo_angle = -180° → pulse MIN (500µs)  → sensor points LEFT extreme

ROS LaserScan convention:
  angle = +π = LEFT extreme,  angle = 0 = FORWARD,  angle = -π = RIGHT extreme
  → ros_angle = -servo_angle

Sweep cycle:
  1. Right extreme (+180° servo / -π ROS) → Left extreme (-180° servo / +π ROS) → publish /scan
  2. Left extreme → Right extreme → publish /scan
  3. Repeat
"""

import math
import random
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, JointState

try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    pigpio = None
    PIGPIO_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO = None
    GPIO_AVAILABLE = False

SERVO_PIN = 12
TRIG_PIN  = 23
ECHO_PIN  = 25

RANGE_MIN = 0.10  # m — HC-SR04 min reliable range
RANGE_MAX = 4.00  # m — HC-SR04 datasheet max


class RadarNode(Node):
    def __init__(self):
        super().__init__("radar_node")

        self.declare_parameter("sweep_steps",    37)    # number of beams per sweep
        self.declare_parameter("servo_coeff",     6.0)  # µs per degree (tune for your servo)
        self.declare_parameter("settle_ms",      20)    # extra wait after servo ramp before ping
        self.declare_parameter("median_reads",    1)    # pings averaged per beam
        self.declare_parameter("sim_mode",       False)

        self._steps   = self.get_parameter("sweep_steps").value
        self._coeff   = float(self.get_parameter("servo_coeff").value)
        self._settle  = self.get_parameter("settle_ms").value / 1000.0
        self._median  = self.get_parameter("median_reads").value
        self._sim     = self.get_parameter("sim_mode").value

        # Servo command range: -π … +π (clamped to 500-2500µs → full physical sweep)
        self._angle_min = -math.pi
        self._angle_max =  math.pi
        self._angle_inc = (self._angle_max - self._angle_min) / (self._steps - 1)

        # LaserScan published angles: 210° (180° servo + 15° HC-SR04 cone each side)
        self._pub_angle_min = -math.radians(105)
        self._pub_angle_max =  math.radians(105)
        self._pub_angle_inc = (self._pub_angle_max - self._pub_angle_min) / (self._steps - 1)

        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self._js_pub   = self.create_publisher(JointState, "/joint_states", 10)

        # pigpio state
        self._pi              = None
        self._echo_cb_handle  = None
        self._echo_start_tick = None
        self._echo_ready      = threading.Event()
        self._echo_dist       = float("inf")
        self._pulse_us        = 1500

        # RPi.GPIO fallback state
        self._servo_pwm = None

        self._init_hw()

        # 10 Hz joint state keeps servo_joint → laser_link TF alive for SLAM
        self.create_timer(0.1, self._pub_joint_state)

        # Sweep runs in a thread — blocking loop, must not share the ROS spin thread
        self._running = True
        self._thread  = threading.Thread(target=self._sweep_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"radar_node | {self._steps} beams × {math.degrees(self._angle_inc):.1f}° | "
            f"coeff={self._coeff} µs/° | settle={self._settle*1000:.0f} ms | "
            f"{'SIM' if self._sim else 'HW (pigpio)' if self._pi else 'HW (RPi.GPIO)'}"
        )

    # ── Hardware init ─────────────────────────────────────────────────────────

    def _init_hw(self):
        if self._sim:
            return

        if PIGPIO_AVAILABLE:
            try:
                self._pi = pigpio.pi()
                if not self._pi.connected:
                    self._pi = None
                    self.get_logger().error("pigpiod not running — start with: sudo pigpiod")
                else:
                    self._pi.set_mode(TRIG_PIN, pigpio.OUTPUT)
                    self._pi.write(TRIG_PIN, 0)
                    self._pi.set_mode(ECHO_PIN, pigpio.INPUT)
                    self._pi.set_pull_up_down(ECHO_PIN, pigpio.PUD_DOWN)
                    self._echo_cb_handle = self._pi.callback(
                        ECHO_PIN, pigpio.EITHER_EDGE, self._echo_handler
                    )
                    self._pi.set_servo_pulsewidth(SERVO_PIN, 1500)
                    self.get_logger().info(
                        f"pigpio OK — SERVO=GPIO{SERVO_PIN} TRIG=GPIO{TRIG_PIN} ECHO=GPIO{ECHO_PIN}"
                    )
                    return
            except Exception as exc:
                self._pi = None
                self.get_logger().error(f"pigpio init failed: {exc}")

        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(TRIG_PIN,  GPIO.OUT)
                GPIO.setup(ECHO_PIN,  GPIO.IN)
                GPIO.output(TRIG_PIN, False)
                GPIO.setup(SERVO_PIN, GPIO.OUT)
                self._servo_pwm = GPIO.PWM(SERVO_PIN, 50)
                self._servo_pwm.start(7.5)
                self.get_logger().warning("RPi.GPIO fallback — ~500 µs jitter on echo timing")
                return
            except Exception as exc:
                self.get_logger().error(f"RPi.GPIO init failed: {exc}")

        self.get_logger().warning("No GPIO available — forcing simulation mode")
        self._sim = True

    # ── pigpio echo callback (runs in pigpiod thread) ─────────────────────────

    def _echo_handler(self, gpio, level, tick):
        if level == 1:
            self._echo_start_tick = tick
        elif level == 0 and self._echo_start_tick is not None:
            elapsed_us = pigpio.tickDiff(self._echo_start_tick, tick)
            self._echo_dist       = elapsed_us * 343.0 / 2_000_000
            self._echo_start_tick = None
            self._echo_ready.set()

    # ── Servo control ─────────────────────────────────────────────────────────

    def _move_servo(self, servo_deg: float):
        """
        Move servo to servo_deg (+90 = right, -90 = left).
        Smooth 5-step ramp over 50 ms to reduce jitter and load on the servo.
        """
        target_us = int(max(500.0, min(2500.0, 1500.0 + servo_deg * self._coeff)))
        if self._pi is not None:
            for i in range(1, 6):
                us = int(self._pulse_us + (target_us - self._pulse_us) * i / 5)
                self._pi.set_servo_pulsewidth(SERVO_PIN, us)
                time.sleep(0.010)
            self._pulse_us = target_us
        elif self._servo_pwm is not None:
            self._servo_pwm.ChangeDutyCycle(target_us / 20_000.0 * 100.0)
            self._pulse_us = target_us

    # ── HC-SR04 ranging ───────────────────────────────────────────────────────

    def _ping(self) -> float:
        if self._sim or (self._pi is None and self._servo_pwm is None):
            time.sleep(0.01)
            return 0.5 + random.gauss(0, 0.02)
        if self._pi is not None:
            return self._ping_pigpio()
        return self._ping_gpio()

    def _ping_pigpio(self) -> float:
        self._echo_ready.clear()
        self._echo_start_tick = None
        self._echo_dist = float("inf")
        self._pi.write(TRIG_PIN, 0)
        time.sleep(0.015)
        self._pi.write(TRIG_PIN, 1)
        time.sleep(0.000010)
        self._pi.write(TRIG_PIN, 0)
        if self._echo_ready.wait(timeout=0.20):
            return self._echo_dist
        self.get_logger().warning("HC-SR04 echo timeout (sensor disconnected?)")
        return float("inf")

    def _ping_gpio(self) -> float:
        GPIO.output(TRIG_PIN, False)
        time.sleep(0.015)
        GPIO.output(TRIG_PIN, True)
        time.sleep(0.000010)
        GPIO.output(TRIG_PIN, False)
        t0 = time.time()
        while GPIO.input(ECHO_PIN) == 0:
            if time.time() - t0 > 0.10:
                return float("inf")
        start = time.time()
        while GPIO.input(ECHO_PIN) == 1:
            if time.time() - start > 0.20:
                return float("inf")
        return (time.time() - start) * 343.0 / 2.0

    def _measure(self, servo_deg: float) -> float:
        """Move to angle, settle, take median of N pings, return clipped distance."""
        self._move_servo(servo_deg)
        time.sleep(self._settle)
        samples = []
        for i in range(self._median):
            if i > 0:
                time.sleep(0.020)
            d = self._ping()
            if RANGE_MIN <= d <= RANGE_MAX:
                samples.append(d)
        if not samples:
            return float("inf")
        samples.sort()
        return samples[len(samples) // 2]

    # ── Sweep loop ────────────────────────────────────────────────────────────

    def _sweep_loop(self):
        # Pre-compute arrays once
        # ros_angles[i]   : ROS angle for beam i  (-π/2 … +π/2)
        # servo_degs[i]   : servo angle for beam i (+90 … -90)
        ros_angles = [self._angle_min + i * self._angle_inc for i in range(self._steps)]
        servo_degs = [-math.degrees(a) for a in ros_angles]
        # servo_degs[0]  = +90° (right)
        # servo_degs[-1] = -90° (left)

        # Move to start position before first sweep
        self._move_servo(servo_degs[0])
        time.sleep(0.5)

        going_right_to_left = True  # first sweep direction

        while self._running and rclpy.ok():
            ranges = [float("inf")] * self._steps
            sweep_start = time.time()

            if going_right_to_left:
                # Sweep: right (+90°) → left (-90°)
                # Visits indices 0 → N-1 in order → fills ranges[0..N-1] directly
                for i in range(self._steps):
                    ranges[i] = self._measure(servo_degs[i])
            else:
                # Sweep: left (-90°) → right (+90°)
                # Visits indices N-1 → 0 in reverse → fills ranges[N-1..0] directly
                for i in reversed(range(self._steps)):
                    ranges[i] = self._measure(servo_degs[i])

            self._publish(ranges, time.time() - sweep_start)
            going_right_to_left = not going_right_to_left

    # ── LaserScan publication ─────────────────────────────────────────────────

    def _publish(self, ranges: list, sweep_time: float):
        valid = sum(1 for r in ranges if math.isfinite(r))
        if valid == 0:
            self.get_logger().warning(f"0 / {self._steps} valid readings — scan not published")
            return

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_link"
        msg.angle_min       = float(self._pub_angle_min)
        msg.angle_max       = float(self._pub_angle_max)
        msg.angle_increment = float(self._pub_angle_inc)
        msg.scan_time       = float(sweep_time)
        msg.time_increment  = float(sweep_time) / max(self._steps - 1, 1)
        msg.range_min       = RANGE_MIN
        msg.range_max       = RANGE_MAX
        msg.ranges          = [float(r) for r in reversed(ranges)]

        self._scan_pub.publish(msg)
        self.get_logger().info(
            f"LaserScan: {valid}/{self._steps} valid | sweep={sweep_time:.2f}s"
        )

    # ── Joint state (keeps servo_joint TF alive) ──────────────────────────────

    def _pub_joint_state(self):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name     = ["servo_joint"]
        js.position = [0.0]
        self._js_pub.publish(js)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        if self._echo_cb_handle:
            try:
                self._echo_cb_handle.cancel()
            except Exception:
                pass
        if self._pi:
            try:
                self._pi.set_servo_pulsewidth(SERVO_PIN, 0)
                self._pi.stop()
            except Exception:
                pass
        if self._servo_pwm:
            try:
                self._servo_pwm.stop()
                GPIO.cleanup()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
